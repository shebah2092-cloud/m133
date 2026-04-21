#!/usr/bin/env python3
"""Simple script to convert a single .dat/.TXT file to CSV"""
import sys
import os

# Add current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from convert_aero_to_csv import detect_file_type, parse_ca_file, create_ca_3d_csv

if __name__ == '__main__':
    # File to convert
    file_path = r"d:\work\6DOF_A\6DOF_A\data\rocket_models\rocket_canard_warhead_ZOH_S\Stage1\CF4T35.TXT"
    
    print("=" * 60)
    print("Converting single file to CSV")
    print("=" * 60)
    print(f"File: {file_path}")
    print()
    
    # Detect file type
    file_type = detect_file_type(file_path)
    print(f"Detected file type: {file_type}")
    
    if file_type == 'ca':
        print("\nParsing CA file...")
        alpha_values, mach_values, reynolds_values, ca_data = parse_ca_file(file_path)
        print(f"  Alpha values: {len(alpha_values)} values")
        print(f"  Mach values: {len(mach_values)} values")
        print(f"  Reynolds values: {len(reynolds_values)} values")
        print(f"  CA blocks: {list(ca_data.keys())}")
        
        # Create output file
        output_file = file_path.replace('.TXT', '.csv').replace('.txt', '.csv').replace('.dat', '.csv')
        output_file = output_file.replace('CF4T35', 'ca_3d_coeffs_CF4T35')
        print(f"\nCreating CSV file: {output_file}")
        create_ca_3d_csv(alpha_values, mach_values, reynolds_values, ca_data, output_file)
        print("Done!")
    else:
        print(f"File type '{file_type}' not supported in this simple script.")
        print("Please use the full convert_aero_to_csv.py script.")


