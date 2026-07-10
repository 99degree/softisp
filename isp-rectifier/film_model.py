#!/usr/bin/env python3
"""
FiLM-Conditioned ISP Distilled Model
=====================================
Skin-tone-aware ISP parameter prediction using Feature-wise Linear Modulation (FiLM).

Architecture:
- Full-frame histogram + metadata → SkinToneEstimator → s ∈ [0,1]
- Shared backbone with FiLM modulation at each conv layer
- Output heads (optionally FiLM-conditioned)
- Single ONNX export, no face detection required

Usage:
    from film_model import FiLMISPDistilledModel, export_film_onnx
    
    model = FiLMISPDistilledModel(metadata_dim=52)
    wb, ccm, tone, zoom, skin_tone = model(hist, meta)
    
    export_film_onnx(model, "fusedispcontroller_film.onnx", metadata_dim=52)
"""

import torch
import torch.nn as nn
from typing import Dict, Tuple, Optional
import os


class SkinToneEstimator(nn.Module):
    """
    Estimates scene skin tone from full-frame histogram + metadata.
    Output: scalar s ∈ [0, 1] where 0 = light skin, 1 = dark skin.
    """
    
    def __init__(self, input_dim: int = 308, hidden: int = 64):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, hidden),
            nn.ReLU(inplace=True),
            nn.Linear(hidden, hidden),
            nn.ReLU(inplace=True),
            nn.Linear(hidden, 1),
            nn.Sigmoid(),  # [0, 1] continuous
        )
    
    def forward(self, histogram: torch.Tensor, metadata: torch.Tensor) -> torch.Tensor:
        """histogram: [B, 256], metadata: [B, meta_dim] → [B, 1]"""
        x = torch.cat([histogram, metadata], dim=1)
        return self.net(x)


class FiLMGenerator(nn.Module):
    """
    Generates γ, β for a single layer from skin tone scalar.
    """
    
    def __init__(self, num_channels: int, hidden: int = 32):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(1, hidden),
            nn.ReLU(inplace=True),
            nn.Linear(hidden, num_channels * 2),
        )
    
    def forward(self, skin_tone: torch.Tensor) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        skin_tone: [B, 1] → gamma: [B, C], beta: [B, C]
        """
        gamma_beta = self.net(skin_tone)  # [B, 2*C]
        gamma, beta = gamma_beta.chunk(2, dim=-1)
        return gamma, beta


class FiLMConvBlock(nn.Module):
    """
    Conv1D + BN + ReLU + FiLM modulation.
    """
    
    def __init__(
        self,
        in_ch: int,
        out_ch: int,
        kernel: int,
        stride: int = 1,
        pool: Optional[int] = None,
        film_hidden: int = 32,
    ):
        super().__init__()
        self.conv = nn.Conv1d(
            in_ch, out_ch, kernel,
            stride=stride, padding=kernel // 2, bias=False
        )
        self.bn = nn.BatchNorm1d(out_ch)
        self.relu = nn.ReLU(inplace=True)
        self.pool = nn.MaxPool1d(pool) if pool else None
        self.film = FiLMGenerator(out_ch, film_hidden)
    
    def forward(self, x: torch.Tensor, skin_tone: torch.Tensor) -> torch.Tensor:
        # x: [B, C_in, L]
        x = self.conv(x)
        x = self.bn(x)
        
        # FiLM modulation
        gamma, beta = self.film(skin_tone)  # [B, C_out] each
        x = gamma.unsqueeze(-1) * x + beta.unsqueeze(-1)  # broadcast over L
        
        x = self.relu(x)
        if self.pool:
            x = self.pool(x)
        return x


class FiLMHistBackbone(nn.Module):
    """
    Histogram backbone with FiLM modulation at each conv layer.
    Returns both features and skin_tone for head conditioning.
    """
    
    def __init__(
        self,
        metadata_dim: int = 52,
        base_ch: int = 16,
        film_hidden: int = 32,
    ):
        super().__init__()
        
        # Skin tone estimator (shared)
        self.skin_tone_est = SkinToneEstimator(256 + metadata_dim)
        
        # Conv blocks with FiLM
        self.block1 = FiLMConvBlock(1, base_ch, 7, pool=2, film_hidden=film_hidden)      # 256→128
        self.block2 = FiLMConvBlock(base_ch, base_ch * 2, 5, pool=2, film_hidden=film_hidden)  # 128→64
        self.block3 = FiLMConvBlock(base_ch * 2, base_ch * 4, 3, film_hidden=film_hidden)
        self.pool = nn.AdaptiveAvgPool1d(32)
        self.flatten = nn.Flatten()
        
        self.out_dim = base_ch * 4 * 32  # 2048 for base_ch=16
    
    def forward(
        self,
        histogram: torch.Tensor,
        metadata: torch.Tensor,
    ) -> Tuple[torch.Tensor, torch.Tensor]:
        """
        histogram: [B, 256], metadata: [B, meta_dim]
        Returns: features [B, 2048], skin_tone [B, 1]
        """
        skin_tone = self.skin_tone_est(histogram, metadata)  # [B, 1]
        
        x = histogram.unsqueeze(1)  # [B, 1, 256]
        x = self.block1(x, skin_tone)
        x = self.block2(x, skin_tone)
        x = self.block3(x, skin_tone)
        x = self.pool(x)
        x = self.flatten(x)  # [B, 2048]
        
        return x, skin_tone


class FiLMHead(nn.Module):
    """
    Output head with optional FiLM conditioning on final layer.
    """
    
    def __init__(
        self,
        in_dim: int,
        hidden: int,
        out_dim: int,
        use_film: bool = True,
        film_hidden: int = 32,
    ):
        super().__init__()
        self.use_film = use_film
        
        self.net = nn.Sequential(
            nn.Linear(in_dim, hidden),
            nn.ReLU(inplace=True),
            nn.Linear(hidden, out_dim),
        )
        
        if use_film:
            self.film = FiLMGenerator(out_dim, film_hidden)
    
    def forward(self, x: torch.Tensor, skin_tone: torch.Tensor) -> torch.Tensor:
        out = self.net(x)  # [B, out_dim]
        if self.use_film:
            gamma, beta = self.film(skin_tone)  # [B, out_dim] each
            out = gamma * out + beta
        return out


class FiLMISPDistilledModel(nn.Module):
    """
    FiLM-Conditioned Distilled ISP Controller Model.
    
    Input:
        - histogram: (B, 256)  # 256-bin luminance histogram
        - metadata: (B, N)     # N=52 for v1.3 (256+52=308 total)
    
    Output:
        - wbgains: (B, 3)      # WB gains [R, G, B]
        - ccm: (B, 9)          # CCM matrix (flattened 3x3)
        - tonecurve: (B, 7)    # Tone curve parameters [0,1]
        - zoom_factor: (B, 1)  # Zoom factor [1,∞)
        - skin_tone: (B, 1)    # Estimated skin tone [0,1]
    
    Args:
        metadata_dim: Metadata feature dimension (default 52 for v1.3)
        base_ch: Base channel count for histogram backbone
        film_hidden: Hidden size for FiLM generators
    """
    
    def __init__(
        self,
        metadata_dim: int = 52,
        base_ch: int = 16,
        film_hidden: int = 32,
    ):
        super().__init__()
        
        # FiLM-enabled histogram backbone
        self.hist_backbone = FiLMHistBackbone(
            metadata_dim=metadata_dim,
            base_ch=base_ch,
            film_hidden=film_hidden,
        )
        
        # Metadata backbone (standard MLP, no FiLM)
        self.meta_backbone = nn.Sequential(
            nn.Linear(metadata_dim, 128),
            nn.BatchNorm1d(128),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
            nn.Linear(128, 256),
            nn.BatchNorm1d(256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
        )
        
        # Fusion
        hist_out_dim = self.hist_backbone.out_dim  # 2048
        meta_out_dim = 256
        combined_dim = hist_out_dim + meta_out_dim  # 2304
        
        self.fusion = nn.Sequential(
            nn.Linear(combined_dim, 512),
            nn.BatchNorm1d(512),
            nn.ReLU(inplace=True),
            nn.Dropout(0.2),
            nn.Linear(512, 256),
            nn.BatchNorm1d(256),
            nn.ReLU(inplace=True),
            nn.Dropout(0.1),
        )
        
        # FiLM-conditioned output heads
        self.wb_head = FiLMHead(256, 64, 3, use_film=True, film_hidden=film_hidden)
        self.ccm_head = FiLMHead(256, 128, 9, use_film=True, film_hidden=film_hidden)
        self.tone_head = FiLMHead(256, 64, 7, use_film=True, film_hidden=film_hidden)
        self.zoom_head = FiLMHead(256, 32, 1, use_film=True, film_hidden=film_hidden)
    
    def forward(
        self,
        histogram: torch.Tensor,
        metadata: torch.Tensor,
    ) -> Dict[str, torch.Tensor]:
        """
        histogram: [B, 256], metadata: [B, meta_dim]
        Returns dict with wb, ccm, tone, zoom, skin_tone
        """
        # FiLM backbone
        hist_features, skin_tone = self.hist_backbone(histogram, metadata)
        
        # Metadata backbone
        meta_features = self.meta_backbone(metadata)
        
        # Fusion
        combined = torch.cat([hist_features, meta_features], dim=1)
        fused = self.fusion(combined)
        
        # FiLM-conditioned heads
        return {
            "wbgains": self.wb_head(fused, skin_tone),
            "ccm": self.ccm_head(fused, skin_tone),
            "tonecurve": torch.sigmoid(self.tone_head(fused, skin_tone)),
            "zoom_factor": torch.relu(self.zoom_head(fused, skin_tone)) + 1.0,
            "skin_tone": skin_tone,  # [B, 1] in [0, 1]
        }
    
    @torch.no_grad()
    def estimate_skin_tone(
        self,
        histogram: torch.Tensor,
        metadata: torch.Tensor,
    ) -> torch.Tensor:
        """Standalone skin tone estimation."""
        return self.hist_backbone.skin_tone_est(histogram, metadata)


def export_film_onnx(
    model: FiLMISPDistilledModel,
    output_path: str,
    metadata_dim: int,
    opset: int = 17,
) -> str:
    """Export FiLM model to ONNX with named outputs."""
    model.eval()
    
    # Dummy inputs
    dummy_hist = torch.randn(1, 256)
    dummy_meta = torch.randn(1, metadata_dim)
    
    # Forward to get output names
    output = model(dummy_hist, dummy_meta)
    output_names = list(output.keys())
    
    dynamic_axes = {
        "histogram": {0: "batch"},
        "metadata": {0: "batch"},
    }
    for name in output_names:
        dynamic_axes[name] = {0: "batch"}
    
    torch.onnx.export(
        model,
        (dummy_hist, dummy_meta),
        output_path,
        input_names=["histogram", "metadata"],
        output_names=output_names,
        dynamic_axes=dynamic_axes,
        opset_version=opset,
        do_constant_folding=True,
    )
    
    # Verify and re-save self-contained
    import onnx
    onnx_model = onnx.load(output_path)
    onnx.checker.check_model(onnx_model)
    onnx.save(onnx_model, output_path)
    
    size_kb = os.path.getsize(output_path) / 1024
    print(f"✅ FiLM model exported: {output_path} ({size_kb:.1f} KB)")
    return output_path


def count_parameters(model: nn.Module) -> int:
    """Count trainable parameters."""
    return sum(p.numel() for p in model.parameters() if p.requires_grad)


if __name__ == "__main__":
    # Quick test
    print("Testing FiLM ISP Model...")
    
    model = FiLMISPDistilledModel(metadata_dim=52)
    n_params = count_parameters(model)
    model_name = model.__class__.__name__
    print(f"{model_name} initialized")
    print(f"Parameters: {n_params:,}")
    
    # Test forward
    hist = torch.randn(2, 256)
    meta = torch.randn(2, 52)
    
    with torch.no_grad():
        output = model(hist, meta)
    
    for k, v in output.items():
        print(f"  {k}: {v.shape}")
    
    # Test export
    import tempfile
    with tempfile.NamedTemporaryFile(suffix=".onnx", delete=False) as f:
        export_film_onnx(model, f.name, metadata_dim=52)
    
    print("\n✅ All tests passed!")