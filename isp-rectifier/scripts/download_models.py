#!/usr/bin/env python3
"""
Download Teacher Model Weights

Downloads pre-trained teacher model weights from various sources.
Supports Google Drive, Hugging Face Hub, and direct URLs.

Usage:
    python scripts/download_models.py --all
    python scripts/download_models.py --ccmnet --awb --isp-tuning
    python scripts/download_models.py --model ccmnet --url https://example.com/ccmnet.pth
"""

import argparse
import hashlib
import os
import sys
import urllib.request
from pathlib import Path
from typing import Dict, Optional
import json

try:
    import gdown
    HAS_GDOWN = True
except ImportError:
    HAS_GDOWN = False

try:
    from huggingface_hub import hf_hub_download
    HAS_HF = True
except ImportError:
    HAS_HF = False


MODEL_CONFIGS = {
    "ccmnet": {
        "filename": "ccmnet.pth",
        "description": "Color Correction Matrix Network",
        "input_dim": 267,
        "output_dim": 9,
        "architecture": "MLP(267->512->256->9)",
        # Add actual URLs when available
        "url": "",  # Direct download URL
        "gdrive_id": "",  # Google Drive file ID
        "hf_repo": "",  # Hugging Face repo (username/model)
        "hf_filename": "ccmnet.pth",
        "sha256": "",  # Expected checksum
    },
    "awb": {
        "filename": "time_aware_awb.pth",
        "description": "Time-Aware Auto White Balance",
        "input_dim": 267,
        "output_dim": 3,
        "architecture": "MLP(267->256->128->3)",
        "url": "",
        "gdrive_id": "",
        "hf_repo": "",
        "hf_filename": "time_aware_awb.pth",
        "sha256": "",
    },
    "isp_tuning": {
        "filename": "neural_isp_tuning.pth",
        "description": "Neural ISP Tuning (Tone + Zoom)",
        "input_dim": 267,
        "output_dim": 8,  # 7 tone + 1 zoom
        "architecture": "MLP(267->512->256->8)",
        "url": "",
        "gdrive_id": "",
        "hf_repo": "",
        "hf_filename": "neural_isp_tuning.pth",
        "sha256": "",
    },
}


def verify_checksum(filepath: Path, expected_sha256: str) -> bool:
    """Verify file SHA256 checksum."""
    if not expected_sha256:
        return True
    
    sha256_hash = hashlib.sha256()
    with open(filepath, "rb") as f:
        for chunk in iter(lambda: f.read(4096), b""):
            sha256_hash.update(chunk)
    
    actual = sha256_hash.hexdigest()
    if actual != expected_sha256:
        print(f"  ❌ Checksum mismatch!")
        print(f"     Expected: {expected_sha256}")
        print(f"     Actual:   {actual}")
        return False
    
    print(f"  ✅ Checksum verified")
    return True


def download_direct(url: str, filepath: Path, show_progress: bool = True) -> bool:
    """Download file from direct URL."""
    try:
        if show_progress:
            def progress_hook(block_num, block_size, total_size):
                downloaded = block_num * block_size
                if total_size > 0:
                    percent = min(100, downloaded * 100 / total_size)
                    print(f"\r  Downloading: {percent:.1f}%", end="", flush=True)
            
            urllib.request.urlretrieve(url, filepath, reporthook=progress_hook)
            print()
        else:
            urllib.request.urlretrieve(url, filepath)
        return True
    except Exception as e:
        print(f"\n  ❌ Download failed: {e}")
        return False


def download_gdrive(file_id: str, filepath: Path) -> bool:
    """Download from Google Drive."""
    if not HAS_GDOWN:
        print("  ❌ gdown not installed. Install with: pip install gdown")
        return False
    
    try:
        gdown.download(id=file_id, output=str(filepath), quiet=False)
        return True
    except Exception as e:
        print(f"  ❌ Google Drive download failed: {e}")
        return False


def download_hf(repo: str, filename: str, filepath: Path) -> bool:
    """Download from Hugging Face Hub."""
    if not HAS_HF:
        print("  ❌ huggingface_hub not installed. Install with: pip install huggingface_hub")
        return False
    
    try:
        hf_hub_download(repo_id=repo, filename=filename, local_dir=str(filepath.parent), local_dir_use_symlinks=False)
        return True
    except Exception as e:
        print(f"  ❌ Hugging Face download failed: {e}")
        return False


def download_model(model_name: str, output_dir: Path, force: bool = False, verify: bool = True) -> bool:
    """Download a single teacher model."""
    if model_name not in MODEL_CONFIGS:
        print(f"  ❌ Unknown model: {model_name}")
        return False
    
    config = MODEL_CONFIGS[model_name]
    filepath = output_dir / config["filename"]
    
    print(f"\n📥 Downloading {config['description']} ({model_name})")
    print(f"   Output: {filepath}")
    
    # Check if already exists
    if filepath.exists() and not force:
        if verify:
            if config["sha256"] and not verify_checksum(filepath, config["sha256"]):
                print("  ⚠️  Existing file has invalid checksum, re-downloading...")
            else:
                print(f"  ✅ Already exists (use --force to re-download)")
                return True
        else:
            print(f"  ✅ Already exists (use --force to re-download)")
            return True
    
    # Try different download methods
    success = False
    
    # 1. Try direct URL
    if config["url"]:
        print(f"   Trying direct URL...")
        success = download_direct(config["url"], filepath)
    
    # 2. Try Google Drive
    if not success and config["gdrive_id"]:
        print(f"   Trying Google Drive...")
        success = download_gdrive(config["gdrive_id"], filepath)
    
    # 3. Try Hugging Face Hub
    if not success and config["hf_repo"]:
        print(f"   Trying Hugging Face Hub...")
        success = download_hf(config["hf_repo"], config["hf_filename"], filepath)
    
    if not success:
        print(f"  ❌ All download methods failed for {model_name}")
        print(f"   Please download manually and place at: {filepath}")
        return False
    
    # Verify
    if verify and config["sha256"]:
        if not verify_checksum(filepath, config["sha256"]):
            filepath.unlink(missing_ok=True)
            return False
    
    print(f"  ✅ Successfully downloaded {model_name}")
    return True


def main():
    parser = argparse.ArgumentParser(
        description="Download teacher model weights",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python scripts/download_models.py --all --output-dir models
  python scripts/download_models.py --ccmnet --awb
  python scripts/download_models.py --model isp_tuning --force
  
Configure models in MODEL_CONFIGS dict or use --url/--gdrive/--hf-repo
"""
    )
    
    parser.add_argument("--output-dir", default="models", help="Output directory")
    parser.add_argument("--force", action="store_true", help="Force re-download")
    parser.add_argument("--no-verify", action="store_true", help="Skip checksum verification")
    parser.add_argument("--list", action="store_true", help="List available models")
    
    # Model selection
    parser.add_argument("--all", action="store_true", help="Download all models")
    parser.add_argument("--ccmnet", action="store_true", help="Download CCMNet")
    parser.add_argument("--awb", action="store_true", help="Download Time-Aware AWB")
    parser.add_argument("--isp-tuning", action="store_true", help="Download Neural ISP Tuning")
    
    # Custom model
    parser.add_argument("--model", help="Custom model name")
    parser.add_argument("--url", help="Direct download URL")
    parser.add_argument("--gdrive", help="Google Drive file ID")
    parser.add_argument("--hf-repo", help="Hugging Face repo (username/model)")
    parser.add_argument("--hf-filename", help="Filename in HF repo")
    parser.add_argument("--sha256", help="Expected SHA256 checksum")
    
    args = parser.parse_args()
    
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    if args.list:
        print("Available models:")
        for name, config in MODEL_CONFIGS.items():
            print(f"  {name:15} - {config['description']}")
            print(f"      Output: {config['filename']}")
            print(f"      Input:  {config['input_dim']}D, Output: {config['output_dim']}D")
        return
    
    # Determine which models to download
    models_to_download = []
    
    if args.all:
        models_to_download = list(MODEL_CONFIGS.keys())
    else:
        if args.ccmnet:
            models_to_download.append("ccmnet")
        if args.awb:
            models_to_download.append("awb")
        if args.isp_tuning:
            models_to_download.append("isp_tuning")
    
    # Custom model
    if args.model:
        if args.model not in MODEL_CONFIGS:
            MODEL_CONFIGS[args.model] = {
                "filename": f"{args.model}.pth",
                "description": f"Custom model: {args.model}",
                "input_dim": 267,
                "output_dim": 1,
                "architecture": "Custom",
                "url": args.url or "",
                "gdrive_id": args.gdrive or "",
                "hf_repo": args.hf_repo or "",
                "hf_filename": args.hf_filename or f"{args.model}.pth",
                "sha256": args.sha256 or "",
            }
        models_to_download.append(args.model)
    
    if not models_to_download:
        parser.print_help()
        print("\n❌ No models specified. Use --all, --ccmnet, --awb, --isp-tuning, or --model")
        sys.exit(1)
    
    # Download models
    print(f"📁 Output directory: {output_dir.absolute()}")
    print(f"🔄 Downloading {len(models_to_download)} model(s)...")
    
    results = {}
    for model in models_to_download:
        results[model] = download_model(
            model, output_dir, 
            force=args.force, 
            verify=not args.no_verify
        )
    
    # Summary
    print("\n" + "="*50)
    print("DOWNLOAD SUMMARY")
    print("="*50)
    
    success_count = 0
    for model, success in results.items():
        status = "✅ SUCCESS" if success else "❌ FAILED"
        print(f"  {model:15} {status}")
        if success:
            success_count += 1
    
    print(f"\nTotal: {success_count}/{len(results)} successful")
    
    if success_count < len(results):
        sys.exit(1)
    
    print("\n✅ All models downloaded successfully!")


if __name__ == "__main__":
    main()