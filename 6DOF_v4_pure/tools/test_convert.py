#!/usr/bin/env python3
"""Test script to convert .dat files to CSV"""
import sys
import os

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from convert_aero_to_csv import process_directory

if __name__ == '__main__':
    # Directory with .TXT files
    target_dir = r"d:\work\6DOF_A\6DOF_A\data\rocket_models\rocket_canard_warhead_ZOH_S\Stage1"
    
    print("=" * 60)
    print("Converting aerodynamic data files to CSV")
    print("=" * 60)
    print(f"Target directory: {target_dir}")
    print()
    
    summary = process_directory(target_dir)
    
    print("\n" + "=" * 60)
    print("Conversion Summary")
    print("=" * 60)
    print(f"CA files: {summary['ca']}")
    print(f"PPPCG files: {summary['pppcg']}")
    print(f"RR files: {summary['rr']}")
    print(f"DATCOM files: {summary['datcom']}")
    if summary['errors']:
        print(f"\nErrors: {len(summary['errors'])}")
        for error in summary['errors']:
            print(f"  - {error}")


