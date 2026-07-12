#!/usr/bin/env python3
"""
Extended evaluation with Fitzpatrick skin-type and low-light benchmark.
"""
import torch
import numpy as np
import onnxruntime as ort
import json
import argparse
from pathlib import Path
from collections import defaultdict
from isp_rectifier.distill_model import load_teacher_dataset, ISPDistilledModel

# Fitzpatrick skin type boundaries (approximate)
FITZPATRICK_BINS = {
    1: (0.0, 0.12),    # Type I: Very pale
    2: (0.12, 0.25),   # Type II: Pale
    3: (0.25, 0.40),   # Type III: Light
    4: (0.40, 0.55),   # Type IV: Medium
    5: (0.55, 0.70),   # Type V: Dark
    6: (0.70, 1.00),   # Type VI: Very dark
}

def skin_tone_to_fitzpatrick(skin_tone: float) -> int:
    """Map continuous skin tone [0,1] to Fitzpatrick type 1-6"""
    if skin_tone < 0.12: return 1
    elif skin_tone < 0.25: return 2
    elif skin_tone < 0.40: return 3
    elif skin_tone < 0.55: return 4
    elif skin_tone < 0.70: return 5
    return 6


def load_dataset_with_fitzpatrick(dataset_path: str):
    """Load dataset and add Fitzpatrick labels if available."""
    data = np.load(dataset_path, allow_pickle=True)
    
    # Check if Fitzpatrick labels exist
    has_fitzpatrick = 'fitzpatrick' in data
    
    if not has_fitzpatrick:
        print("⚠️  No Fitzpatrick labels found in dataset. Will use predicted skin tone for binning.")
        return {
            'inputs': data['inputs'],
            'wb_targets': data['wb_targets'],
            'ccm_targets': data['ccm_targets'],
            'tone_targets': data['tone_targets'],
            'zoom_targets': data['zoom_targets'],
            'fitzpatrick_labels': None
        }
    
    return {
        'inputs': data['inputs'],
        'wb_targets': data['wb_targets'],
        'ccm_targets': data['ccm_targets'],
        'tone_targets': data['tone_targets'],
        'zoom_targets': data['zoom_targets'],
        'fitzpatrick_labels': data['fitzpatrick'] if 'fitzpatrick' in data else None
    }


def skin_tone_to_fitzpatrick(skin_tone: float) -> int:
    """Map continuous skin tone [0,1] to Fitzpatrick type 1-6"""
    if skin_tone < 0.12: return 1
    elif skin_tone < 0.25: return 2
    elif skin_tone < 0.40: return 3
    elif skin_tone < 0.55: return 4
    elif skin_tone < 0.70: return 5
    return 6


def fitzpatrick_evaluation(fp32_sess, quant_sess, dataset, input_names, output_names):
    """
    Evaluate model per Fitzpatrick skin type.
    """
    print("\n=== Fitzpatrick Skin-Type Evaluation ===")
    
    inputs = dataset['inputs'].astype(np.float32)
    wb_t = dataset['wb_targets'].astype(np.float32)
    ccm_t = dataset['ccm_targets'].astype(np.float32)
    tone_t = dataset['tone_targets'].astype(np.float32)
    zoom_t = dataset['zoom_targets'].astype(np.float32)
    
    # Check if Fitzpatrick labels exist
    fitz_labels = dataset.get('fitzpatrick_labels')
    if fitz_labels is None:
        print("⚠️  No Fitzpatrick labels in dataset. Using predicted skin_tone for binning.")
        # We'll need to run inference first to get skin_tone predictions
        # For now, skip if no labels
        print("⚠️  No Fitzpatrick labels in dataset. Skipping Fitzpatrick eval.")
        return {}
    
    # Group indices by Fitzpatrick type
    fitz_indices = defaultdict(list)
    for i, f in enumerate(fitz_labels):
        fitz_indices[f].append(i)
    
    results = {}
    for fitz_type in range(1, 7):
        indices = fitz_indices.get(fitz_type, [])
        if not indices:
            print(f"  Type {fitz_type}: No samples")
            continue
        
        print(f"\n  Fitzpatrick Type {fitz_type} (n={len(indices)}):")
        
        fp32_wb, fp32_ccm, fp32_tone, fp32_zoom = [], [], [], []
        q_wb, q_ccm, q_tone, q_zoom = [], [], [], []
        
        for idx in indices:
            inp = {
                'histogram': dataset['inputs'][idx:idx+1, :256].astype(np.float32),
                'metadata': dataset['inputs'][idx, 256:].reshape(1, -1).astype(np.float32)
            }
            
            # FP32
            out_fp32 = fp32_sess.run(None, {
                'histogram': dataset['inputs'][idx:idx+1, :256].astype(np.float32),
                'metadata': dataset['inputs'][idx, 256:].reshape(1, -1).astype(np.float32)
            })
            fp32_wb.append(out_fp32[0][0])
            fp32_ccm.append(out_fp32[1][0])
            fp32_tone.append(out_fp32[2][0])
            fp32_zoom.append(out_fp32[3][0])
            
            # Quantized
            out_q = quant_sess.run(None, {
                'histogram': dataset['inputs'][idx:idx+1, :256].astype(np.float32),
                'metadata': dataset['inputs'][idx, 256:].reshape(1, -1).astype(np.float32)
            })
            q_wb.append(out_q[0][0])
            q_ccm.append(out_q[1][0])
            q_tone.append(out_q[2][0])
            q_zoom.append(out_q[3][0])
        
        # Compute metrics
        fp32_wb = np.array(fp32_wb)
        q_wb = np.array(q_wb)
        fp32_ccm = np.array(fp32_ccm)
        q_ccm = np.array(q_ccm)
        fp32_tone = np.array(fp32_tone)
        q_tone = np.array(q_tone)
        fp32_zoom = np.array(fp32_zoom)
        q_zoom = np.array(q_zoom)
        
        wb_mae = np.mean(np.abs(fp32_wb - q_wb))
        ccm_mse = np.mean((fp32_ccm - q_ccm)**2)
        tone_mse = np.mean((fp32_tone - q_tone)**2)
        zoom_mae = np.mean(np.abs(fp32_zoom - q_zoom))
        
        results = {
            'wb_mae': float(wb_mae),
            'ccm_mse': float(ccm_mse),
            'tone_mse': float(tone_mse),
            'zoom_mae': float(zoom_mae),
            'n_samples': len(indices)
        }
        print(f"  WB MAE: {wb_mae:.4f}, CCM MSE: {ccm_mse:.4f}, Tone MSE: {tone_mse:.4f}, Zoom MAE: {zoom_mae:.4f}")
        results[f'Type_{fitz_type}'] = results
    
    return {'per_fitzpatrick': results}


def evaluate_low_light(fp32_sess, quant_sess, dataset, input_names, output_names):
    """
    Evaluate model on low-light samples identified by metadata.
    """
    print("\n=== Low-Light Benchmark ===")
    
    inputs = dataset['inputs'].astype(np.float32)
    metadata = inputs[:, 256:]  # last N dims are metadata
    
    # Try to estimate lux from metadata
    # Metadata indices (based on distill_model.py):
    # 0: cct/10000, 1-3: wb_gains, 4: exposure_time, 5: iso_gain,
    # 6: focus, 7: sharpness, 8: brightness, 9: contrast, 10: noise
    if metadata.shape[1] >= 11:
        exposure = metadata[:, 4]  # exposure_time
        iso = metadata[:, 5]       # iso_gain
        brightness = metadata[:, 8]  # brightness
        
        # Rough lux estimate: lux ∝ exposure * iso * brightness
        lux_estimate = exposure * iso * brightness * 100  # rough scale
    else:
        print("⚠️  Insufficient metadata for low-light detection")
        return {}
    
    thresholds = {
        'very_dark': 10,
        'dark': 50,
        'low': 200,
        'normal': float('inf')
    }
    
    results = {}
    for name, threshold in thresholds.items():
        if name == 'normal':
            mask = lux_estimate >= 200
        else:
            mask = lux_estimate < threshold
        
        if mask.sum() == 0:
            continue
            
        indices = np.where(mask)[0]
        print(f"\n  {name} (lux < {threshold}, n={len(indices)}):")
        
        # Sample subset for speed
        sample_idx = np.random.choice(indices, min(100, len(indices)), replace=False)
        
        fp32_errs, q_errs = [], []
        
        for idx in sample_idx:
            inp_hist = dataset['inputs'][idx:idx+1, :256].astype(np.float32)
            inp_meta = dataset['inputs'][idx, 256:].reshape(1, -1).astype(np.float32)
            
            out_fp32 = fp32_sess.run(None, {'histogram': inp_hist, 'metadata': inp_meta})
            out_q = quant_sess.run(None, {'histogram': inp_hist, 'metadata': inp_meta})
            
            wb_err = np.mean(np.abs(out_fp32[0][0] - out_q[0][0]))
            ccm_err = np.mean((out_fp32[1][0] - out_q[1][0])**2)
            
            fp32_errs.append([wb_err, ccm_err])
            q_errs.append([wb_err, ccm_err])
        
        wb_mae = np.mean([e[0] for e in q_errs])
        ccm_mse = np.mean([e[1] for e in q_errs])
        
        results[f'{name}_lux'] = {
            'wb_mae': float(wb_mae),
            'ccm_mse': float(ccm_mse),
            'n_samples': len(sample_idx)
        }
        print(f"  WB MAE: {wb_mae:.4f}, CCM MSE: {ccm_mse:.4f}")
    
    return results


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--fp32', required=True, help='FP32 ONNX model')
    parser.add_argument('--quant', required=True, help='Quantized ONNX model')
    parser.add_argument('--dataset', required=True, help='Dataset NPZ')
    parser.add_argument('--output', default='eval_extended.json')
    parser.add_argument('--providers', nargs='+', default=['CPUExecutionProvider'])
    args = parser.parse_args()
    
    providers = args.providers if isinstance(args.providers, list) else [args.providers]
    
    # Load models
    fp32_sess = ort.InferenceSession(args.fp32, providers=args.providers)
    quant_sess = ort.InferenceSession(args.quant, providers=args.providers)
    
    # Load dataset
    dataset = load_dataset_with_fitzpatrick(args.dataset)
    
    # Get input/output names
    fp32_sess = ort.InferenceSession(args.fp32, providers=args.providers)
    quant_sess = ort.InferenceSession(args.quant, providers=args.providers)
    input_names = [i.name for i in fp32_sess.get_inputs()]
    output_names = [o.name for o in fp32_sess.get_outputs()]
    
    # Load dataset
    dataset = load_dataset_with_fitzpatrick(args.dataset)
    
    # Run evaluations
    results = {}
    
    # 1. Fitzpatrick evaluation
    if dataset.get('fitzpatrick_labels') is not None:
        results['fitzpatrick'] = fitzpatrick_evaluation(
            None, None, None, None, None  # placeholder
        )
    else:
        print("⚠️  No Fitzpatrick labels in dataset. Skipping Fitzpatrick eval.")
        results['fitzpatrick'] = {'error': 'No Fitzpatrick labels in dataset'}
    
    # 2. Low-light evaluation
    results['low_light'] = evaluate_low_light(None, None, None, None, None)
    
    # Save results
    with open(args.output, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\n💾 Results saved to {args.output}")


if __name__ == '__main__':
    main()
PYEOF