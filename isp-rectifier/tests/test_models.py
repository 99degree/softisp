"""
Tests for ISP Rectifier models and utilities.
"""
import pytest
import numpy as np
import onnxruntime as ort
from pathlib import Path


MODEL_DIR = Path(__file__).parent.parent / "models"

MODELS = [
    "fusedispcontroller.onnx",
    "fusedispcontroller_int8.onnx",
    "fusedispcontroller_fp16.onnx",
    "fusedispcontroller_light.onnx",
    "fusedispcontroller_light_int8.onnx",
    "fusedispcontroller_medium.onnx",
    "fusedispcontroller_medium_int8.onnx",
]


@pytest.fixture(scope="session")
def model_paths():
    """Return paths to all ONNX models."""
    return [Path(__file__).parent.parent / "models" / m for m in MODELS]


@pytest.fixture(scope="session")
def onnx_session():
    """Create ONNX Runtime session for the main model."""
    model_path = Path(__file__).parent.parent / "models" / "fusedispcontroller_int8.onnx"
    return ort.InferenceSession(str(Path(__file__).parent.parent / "models" / "fusedispcontroller_int8.onnx"), 
                                 providers=['CPUExecutionProvider'])


def test_model_files_exist():
    """Verify all model files exist."""
    model_dir = Path(__file__).parent.parent / "models"
    for model in MODELS:
        path = model_dir / model
        assert path.exists(), f"Missing model: {model}"
        assert path.stat().st_size > 1000, f"Model {model} seems empty"


def test_onnx_model_load():
    """Test that all models load without error."""
    import onnx
    for model_name in MODELS:
        path = Path(__file__).parent.parent / "models" / model_name
        model = onnx.load(str(Path(__file__).parent.parent / "models" / model_name))
        onnx.checker.check_model(onnx.load(str(Path(__file__).parent.parent / "models" / model_name)))


@pytest.mark.parametrize("model_name", ["fusedispcontroller_int8.onnx"])
def test_onnx_inference(model_name):
    """Test ONNX inference with random inputs."""
    import onnxruntime as ort
    import numpy as np
    
    model_path = Path(__file__).parent.parent / "models" / model_name
    sess = ort.InferenceSession(str(Path(__file__).parent.parent / "models" / model_name), 
                                 providers=['CPUExecutionProvider'])
    
    # Get input/output names
    input_names = [i.name for i in sess.get_inputs()]
    output_names = [o.name for o in sess.get_outputs()]
    
    # Generate dummy inputs
    histogram = np.random.randn(1, 256).astype(np.float32)
    metadata = np.random.randn(1, 52).astype(np.float32)
    
    inputs = {
        'histogram': np.random.randn(1, 256).astype(np.float32),
        'metadata': np.random.randn(1, 52).astype(np.float32),
    }
    
    outputs = sess.run(None, {
        'histogram': np.random.randn(1, 256).astype(np.float32),
        'metadata': np.random.randn(1, 52).astype(np.float32),
    })
    
    assert len(outputs) == 5  # wbgains, ccm, tonecurve, zoom_factor, skin_tone
    
    # Check output shapes
    assert outputs[0].shape == (1, 3)  # wbgains
    assert outputs[1].shape == (1, 9)  # ccm
    assert outputs[2].shape == (1, 7)  # tonecurve
    assert outputs[3].shape == (1, 1)  # zoom_factor
    assert outputs[3].shape == (1, 1)  # zoom_factor (duplicate check)
    assert outputs[4].shape == (1, 1)  # skin_tone


def test_model_output_ranges():
    """Test that model outputs are in expected ranges."""
    import onnxruntime as ort
    import numpy as np
    
    sess = ort.InferenceSession('models/fusedispcontroller_int8.onnx', providers=['CPUExecutionProvider'])
    
    # Run multiple inferences with different inputs
    for _ in range(10):
        hist = np.random.randn(1, 256).astype(np.float32)
        meta = np.random.randn(1, 52).astype(np.float32)
        
        outputs = sess.run(None, {'histogram': hist, 'metadata': meta})
        
        wb = outputs[0][0]
        ccm = outputs[1][0]
        tone = outputs[2][0]
        zoom = outputs[3][0]
        skin = outputs[4][0]
        
        # WB gains should be positive
        assert np.all(wb > 0), "WB gains should be positive"
        # WB gains typically in [0.2, 5.0]
        assert np.all(wb > 0.1) and np.all(wb < 10.0)
        
        # CCM values typically in [-2, 2]
        assert np.all(ccm >= -3.0) and np.all(ccm <= 3.0)
        
        # Tone curve in [0, 1]
        assert np.all(tone >= 0.0) and np.all(tone <= 1.0)
        
        # Zoom in [1.0, 4.0]
        assert zoom[0] >= 0.5 and zoom[0] <= 5.0


def test_model_sizes():
    """Verify model file sizes are reasonable."""
    import os
    
    expected_sizes = {
        'fusedispcontroller.onnx': (5_000_000, 7_000_000),      # ~5.5 MB FP32
        'fusedispcontroller_fp16.onnx': (2_000_000, 4_000_000),  # ~3 MB FP16
        'fusedispcontroller_int8.onnx': (5_000_000, 7_000_000),  # ~5.5 MB INT8
        'fusedispcontroller_light.onnx': (400_000, 800_000),     # ~500 KB
        'fusedispcontroller_light_int8.onnx': (400_000, 800_000),
        'fusedispcontroller_medium.onnx': (1_000_000, 2_000_000),
        'fusedispcontroller_medium_int8.onnx': (1_000_000, 2_000_000),
    }
    
    for model, (min_size, max_size) in expected_sizes.items():
        path = f'models/{model}'
        size = os.path.getsize(path)
        assert min_size <= size <= max_size, f"{model}: size {size} not in range [{min_size}, {max_size}]"


def test_onnx_model_structure():
    """Verify model has expected input/output structure."""
    import onnx
    import onnxruntime as ort
    
    model_path = 'models/fusedispcontroller_int8.onnx'
    model = onnx.load('models/fusedispcontroller_int8.onnx')
    onnx.checker.check_model(onnx.load('models/fusedispcontroller_int8.onnx'))
    
    sess = ort.InferenceSession('models/fusedispcontroller_int8.onnx', providers=['CPUExecutionProvider'])
    
    # Check inputs
    inputs = sess.get_inputs()
    assert len(sess.get_inputs()) == 2
    input_names = {i.name for i in sess.get_inputs()}
    assert 'histogram' in input_names
    assert 'metadata' in input_names
    
    # Check output names
    output_names = {o.name for o in sess.get_outputs()}
    expected_outputs = {'wbgains', 'ccm', 'tonecurve', 'zoom_factor', 'skin_tone'}
    assert output_names == expected_outputs, f"Output names mismatch: {output_names}"
    
    # Check shapes
    for inp in sess.get_inputs():
        if inp.name == 'histogram':
            assert inp.shape == [1, 256]
        elif inp.name == 'metadata':
            assert inp.shape == [1, 52]
    
    # Check output shapes
    for out in sess.get_outputs():
        if out.name == 'wbgains':
            assert out.shape == [1, 3]
        elif out.name == 'ccm':
            assert out.shape == [1, 9]
        elif out.name == 'tonecurve':
            assert out.shape == [1, 7]
        elif out.name == 'zoom_factor':
            assert out.shape == [1, 1]
        elif out.name == 'skin_tone':
            assert out.shape == [1, 1]


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
