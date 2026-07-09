#!/usr/bin/env python3
"""
Test suite for isp-rectifier components
"""

import pytest
import numpy as np
import torch
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from isp_rectifier.types import FrameMetadata, RectifiedParams, ISPOptimizedParams


class TestTypes:
    """Test type definitions and serialization."""
    
    def test_frame_metadata_creation(self):
        """Test FrameMetadata can be created with all fields."""
        metadata = FrameMetadata(
            histogram=[0]*256,
            cct=5500.0,
            wb_gains=[1.2, 1.0, 0.9],
            ae_exposure_time=0.033,
            ae_iso_gain=2.0,
            af_position=0.5,
            af_sharpness=0.8,
            brightness=0.6,
            contrast=0.7,
            noise_level=0.1,
            timestamp=1234567890,
        )
        
        assert len(metadata.histogram) == 256
        assert metadata.cct == 5500.0
        assert metadata.wb_gains == [1.2, 1.0, 0.9]
    
    def test_frame_metadata_to_vector(self):
        """Test metadata vector conversion."""
        metadata = FrameMetadata(
            histogram=[1.0]*256,
            cct=6500.0,
            wb_gains=[1.0, 1.0, 1.0],
            ae_exposure_time=0.033,
            ae_iso_gain=1.0,
            af_position=0.5,
            af_sharpness=0.5,
            brightness=0.5,
            contrast=0.5,
            noise_level=0.1,
            timestamp=0,
        )
        
        vec = metadata.to_vector()
        assert vec.shape == (267,)
        assert np.allclose(vec[:256], 1.0)
        assert abs(vec[256] - 0.65) < 0.01  # CCT normalized
    
    def test_rectified_params_defaults(self):
        """Test RectifiedParams default values."""
        params = RectifiedParams()
        
        assert params.exposure_time == 0.033
        assert params.iso_gain == 1.0
        assert params.wb_gains == [1.0, 1.0, 1.0]
        assert params.color_correction_matrix == [[1,0,0],[0,1,0],[0,0,1]]
        assert params.tone_map_curve == [0.0, 0.25, 0.5, 0.75, 1.0]
        assert params.sharpness == 1.0
        assert params.noise_reduction == 0.5
    
    def test_isp_optimized_params_clamping(self):
        """Test ISPOptimizedParams clamps to valid ranges."""
        params = ISPOptimizedParams(
            wb_r_gain=15.0,      # Should clamp to 10.0
            wb_g_gain=-0.5,      # Should clamp to 0.1
            wb_b_gain=2.0,
            ccm=[[3.0, -3.0, 0], [0, 1, 0], [0, 0, 1]],  # Should clamp
            tone_curve_lut=[-0.5, 1.5, 0.5],  # Should clamp
            zoom_factor=10.0,    # Should clamp to 4.0
        )
        
        assert params.wb_r_gain == 10.0
        assert params.wb_g_gain == 0.1
        assert params.ccm[0][0] == 2.0
        assert params.ccm[0][1] == -2.0
        assert params.tone_curve_lut[0] == 0.0
        assert params.tone_curve_lut[1] == 1.0
        assert params.zoom_factor == 4.0


class TestModelArchitecture:
    """Test model architecture and forward pass."""
    
    @pytest.fixture
    def model(self):
        """Create model instance."""
        # Import here to avoid torch dependency if not testing
        from distill_model import ISPDistilledModel
        return ISPDistilledModel(metadata_dim=267)
    
    def test_model_forward(self, model):
        """Test model forward pass."""
        model.eval()
        with torch.no_grad():
            histogram = torch.randn(2, 256)
            metadata = torch.randn(2, 267)
            
            outputs = model(histogram, metadata)
            
            assert "wbgains" in outputs
            assert "ccm" in outputs
            assert "tonecurve" in outputs
            assert "zoom_factor" in outputs
            
            assert outputs["wbgains"].shape == (2, 3)
            assert outputs["ccm"].shape == (2, 9)
            assert outputs["tonecurve"].shape == (2, 7)
            assert outputs["zoom_factor"].shape == (2, 1)
    
    def test_model_parameters(self, model):
        """Test model has reasonable parameter count."""
        total_params = sum(p.numel() for p in model.parameters())
        trainable_params = sum(p.numel() for p in model.parameters() if p.requires_grad)
        
        # Should be under 2M parameters for embedded deployment
        assert total_params < 2_000_000
        assert trainable_params == total_params
        
        print(f"Total params: {total_params:,}")
        print(f"Trainable params: {trainable_params:,}")


class TestDistillationLoss:
    """Test distillation loss computation."""
    
    def test_loss_computation(self):
        """Test distillation loss between student and teacher."""
        from distill_model import distillation_loss
        
        student = {
            "wbgains": torch.tensor([[1.1, 1.0, 0.9]]),
            "ccm": torch.eye(3).flatten().unsqueeze(0),
            "tonecurve": torch.linspace(0, 1, 7).unsqueeze(0),
            "zoom_factor": torch.tensor([[1.0]]),
        }
        
        teacher = {
            "wbgains": torch.tensor([[1.0, 1.0, 1.0]]),
            "ccm": torch.eye(3).flatten().unsqueeze(0),
            "tonecurve": torch.linspace(0, 1, 7).unsqueeze(0),
            "zoom_factor": torch.tensor([[1.0]]),
        }
        
        loss = distillation_loss(student, teacher)
        
        assert loss.item() > 0
        assert loss.item() < 1.0
    
    def test_loss_with_weights(self):
        """Test loss with custom weights."""
        from distill_model import distillation_loss
        
        student = {"wbgains": torch.tensor([[1.5, 1.5, 1.5]])}
        teacher = {"wbgains": torch.tensor([[1.0, 1.0, 1.0]])}
        
        # High weight on WB
        loss_high = distillation_loss(student, teacher, {"wbgains": 10.0})
        # Low weight on WB
        loss_low = distillation_loss(student, teacher, {"wbgains": 0.1})
        
        assert loss_high > loss_low


class TestONNXExport:
    """Test ONNX export functionality."""
    
    @pytest.mark.skipif(not torch.cuda.is_available(), reason="CUDA not available")
    def test_export_onnx(self, tmp_path):
        """Test model exports to valid ONNX."""
        from distill_model import ISPDistilledModel, export_to_onnx
        
        model = ISPDistilledModel(metadata_dim=267)
        model.eval()
        
        dummy_hist = torch.randn(1, 256)
        dummy_meta = torch.randn(1, 267)
        
        onnx_path = tmp_path / "test_model.onnx"
        export_to_onnx(model, str(onnx_path), (dummy_hist, dummy_meta))
        
        assert onnx_path.exists()
        
        # Verify with onnx
        import onnx
        onnx_model = onnx.load(str(onnx_path))
        onnx.checker.check_model(onnx_model)
        
        # Check inputs/outputs
        input_names = [i.name for i in onnx_model.graph.input]
        output_names = [o.name for o in onnx_model.graph.output]
        
        assert "histogram" in input_names
        assert "metadata" in input_names
        assert "wbgains" in output_names
        assert "ccm" in output_names
        assert "tonecurve" in output_names
        assert "zoom_factor" in output_names


class TestRegisterInjector:
    """Test register injection logic."""
    
    def test_inject_registers_clamping(self):
        """Test register injection clamps values."""
        # This would test the Rust register_injector logic
        # For Python test, we test the Python equivalent
        from scripts.test_register_injector import inject_parameters_python
        
        params = ISPOptimizedParams(
            wb_r_gain=15.0,
            wb_g_gain=15.0,
            wb_b_gain=15.0,
            ccm=[[3.0]*3]*3,
            tone_curve_lut=[2.0]*7,
            zoom_factor=10.0,
        )
        
        # Should clamp all values
        registers = inject_parameters_python(params)
        
        assert registers.wb_r_gain <= 10.0
        assert registers.wb_g_gain <= 10.0
        assert registers.wb_b_gain <= 10.0
        assert all(abs(v) <= 2.0 for row in registers.ccm for v in row)
        assert all(0.0 <= v <= 1.0 for v in registers.tone_curve_lut)
        assert registers.zoom_factor <= 4.0


# ============================================================================
# Fixtures and utilities
# ============================================================================

@pytest.fixture
def sample_metadata():
    """Sample FrameMetadata for testing."""
    return FrameMetadata(
        histogram=np.random.randint(0, 256, 256).tolist(),
        cct=5500.0,
        wb_gains=[1.1, 1.0, 0.95],
        ae_exposure_time=0.033,
        ae_iso_gain=2.0,
        af_position=0.5,
        af_sharpness=0.75,
        brightness=0.55,
        contrast=0.65,
        noise_level=0.08,
        timestamp=1234567890,
    )


if __name__ == "__main__":
    pytest.main([__file__, "-v"])