#!/usr/bin/env python3
"""
Convert aerodynamics .dat files to CSV format matching rocket_models structure.

This script automatically detects and processes aerodynamic data files. It can work
in two modes:
1. Single directory mode (default): Process only the directory where the script is located
2. Multi-directory mode (--all): Process all sibling directories that contain aerodynamic data

The script identifies file types based on their content structure, not their filenames.

Supported file types (auto-detected):
1. CA files: 3D axial force coefficients (alpha, Mach, Reynolds)
   - Detected by: First line has 3 integers, contains "CA" blocks
   - Output: ca_3d_coeffs_[index].csv

2. PPPCG files: Multiple 2D coefficient tables (alpha, Mach)
   - Detected by: Contains "alpha (rows):" and "mach (columns):" headers
   - Contains: CN-A, CNd, Cma, CMd, CNq, CMq, CLLP, Cad
   - Output: aero_coeffs.csv, damping_coeffs.csv, fin_deflection_coeffs.csv

3. RR files: Roll moment coefficients (alpha, Mach, roll_deflection)
   - Detected by: First line has 3 integers, contains "CLL" blocks
   - Output: roll_aero_coeffs.csv

Target CSV files:
- aero_coeffs.csv: Mach, alpha_deg, Cd, Cn, Cm
- ca_3d_coeffs_[N].csv: Mach, alpha_deg, Reynolds, CA
- damping_coeffs.csv: Mach, alpha_deg, Cmq, Cnq, Clp
- roll_aero_coeffs.csv: Mach, alpha_deg, def_roll, Cll
- fin_deflection_coeffs.csv: Mach, alpha_deg, CNd, CMd

Usage:
    python convert_aero_to_csv.py           # Process current directory only
    python convert_aero_to_csv.py --all     # Process all sibling directories
    python convert_aero_to_csv.py --dir /path/to/dir  # Process specific directory
"""

import os
import re
import glob
import argparse
import numpy as np
import pandas as pd

try:
    import openpyxl
    EXCEL_SUPPORT = True
except ImportError:
    EXCEL_SUPPORT = False


def detect_file_type(filepath):
    """Detect the type of aerodynamic data file based on its content.
    
    Returns:
        str: 'ca' for CA coefficient files (3D: alpha, Mach, Reynolds)
             'pppcg' for PPPCG files (2D coefficient tables)
             'rr' for RR files (roll coefficients)
             'datcom' for MISSILE DATCOM output files
             None if file type cannot be determined
    """
    try:
        with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
            content = f.read()
            lines = content.split('\n')
    except Exception:
        return None
    
    # Skip empty files
    if not content.strip():
        return None
    
    content_lower = content.lower()
    
    # Check for PPPCG file (has alpha/mach headers)
    # Support various formats: "alpha (rows):", "alpha(rows) :", "alpha(rows):", etc.
    has_alpha = bool(re.search(r'alpha\s*\(rows\)\s*:', content_lower))
    # Support "mach (columns):", "mach(colomns) :", "mach(columns):", etc. (including typo "colomns")
    has_mach = bool(re.search(r'mach\s*\(col[ou]m[n]?s\)\s*:', content_lower))
    
    # Also support "Alpha:" and "Mach:" format (simple colon format)
    # دعم تنسيق "Alpha:" و "Mach:" (تنسيق النقطتين البسيط)
    if not has_alpha:
        has_alpha = bool(re.search(r'^alpha\s*:', content_lower, re.MULTILINE))
    if not has_mach:
        has_mach = bool(re.search(r'^mach\s*:', content_lower, re.MULTILINE))
    
    if has_alpha and has_mach:
        return 'pppcg'
    
    # Check for simplified format: starts with CMA, CNA, CMQA, CNQA blocks
    # التحقق من التنسيق المبسط: يبدأ بكتل CMA, CNA, CMQA, CNQA
    has_cma = bool(re.search(r'^\s*CMA\s*$', content, re.MULTILINE))
    has_cna = bool(re.search(r'^\s*CNA\s*$', content, re.MULTILINE))
    has_cmqa = bool(re.search(r'^\s*CMQA\s*$', content, re.MULTILINE) or re.search(r'^\s*CMQ\s*$', content, re.MULTILINE))
    has_cnqa = bool(re.search(r'^\s*CNQA\s*$', content, re.MULTILINE) or re.search(r'^\s*CNQ\s*$', content, re.MULTILINE))
    
    if has_cma and has_cna and has_cmqa and has_cnqa:
        return 'pppcg'
    
    # Check for DATCOM output file (has "No of BETA is:" header and coefficient blocks)
    if re.search(r'no\s+of\s+beta\s+is\s*:', content_lower):
        # Verify it has coefficient blocks like CN, CM, CA
        has_cn = bool(re.search(r'^\s*CN\s*$', content, re.MULTILINE))
        has_cm = bool(re.search(r'^\s*CM\s*$', content, re.MULTILINE))
        has_ca = bool(re.search(r'^\s*CA\s*$', content, re.MULTILINE))
        if has_cn and has_cm and has_ca:
            return 'datcom'
    
    # Check for CA or RR file (starts with 3 integers)
    first_line = None
    for line in lines:
        line = line.strip()
        if line:
            first_line = line
            break
    
    if first_line:
        parts = first_line.split()
        if len(parts) >= 3:
            try:
                # Check if first 3 values are integers
                int(parts[0])
                int(parts[1])
                int(parts[2])
                
                # Now check if it has CA, REN (Reynolds), or CLL blocks
                # CA files can have "CA1e5", "REN1e5", "CA-A0.0", "CA-A10000.0", etc. block headers
                if (re.search(r'\bCA\d', content) or re.search(r'\bREN\d', content) or 
                    re.search(r'\bCA-A\d', content, re.IGNORECASE)):
                    return 'ca'
                elif re.search(r'\bCLL\d', content):
                    return 'rr'
            except ValueError:
                pass
    
    return None


def find_data_files(directory):
    """Find and categorize all aerodynamic data files in a directory.
    
    Args:
        directory: Path to search for data files
        
    Returns:
        dict: Dictionary with keys 'ca', 'pppcg', 'rr', 'datcom' containing lists of file paths
    """
    files = {'ca': [], 'pppcg': [], 'rr': [], 'datcom': []}
    
    # Search for .dat files and other potential data files
    for pattern in ['*.dat', '*.DAT', '*.txt', '*.TXT']:
        for filepath in glob.glob(os.path.join(directory, pattern)):
            file_type = detect_file_type(filepath)
            if file_type:
                files[file_type].append(filepath)
    
    # Also check subdirectories one level deep
    for subdir in os.listdir(directory):
        subdir_path = os.path.join(directory, subdir)
        if os.path.isdir(subdir_path):
            for pattern in ['*.dat', '*.DAT', '*.txt', '*.TXT']:
                for filepath in glob.glob(os.path.join(subdir_path, pattern)):
                    file_type = detect_file_type(filepath)
                    if file_type:
                        files[file_type].append(filepath)
    
    return files


def parse_ca_file(filepath):
    """Parse CA coefficient file format (3D: alpha, Mach, Reynolds).
    
    Format:
    Line 1: num_alpha, num_mach, num_reynolds (3 integers)
    Line 2: alpha values (space/tab separated)
    Line 3: mach values
    Line 4: reynolds values
    Then blocks of CA data for each Reynolds number
    Block headers can be "CA1e5", "REN1e5", etc.
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # Parse header - note: order is num_alpha, num_mach, num_reynolds
    header = lines[0].strip().split()
    num_alpha = int(header[0])
    num_mach = int(header[1])
    num_reynolds = int(header[2])
    
    # Parse alpha values
    alpha_values = [float(x) for x in lines[1].strip().split()]
    
    # Parse mach values
    mach_values = [float(x) for x in lines[2].strip().split()]
    
    # Parse reynolds values
    reynolds_values = [float(x) for x in lines[3].strip().split()]
    
    # Parse CA data blocks
    ca_data = {}
    current_reynolds = None
    data_lines = []
    
    for line in lines[4:]:
        line = line.strip()
        if not line:
            continue
        
        # Check if this is a Reynolds header (e.g., "CA1e+05", "REN1e5", "REN5e5", "CA-A0.0", "CA-A10000.0")
        reynolds_match = re.match(r'^(?:CA|REN|CA-A)(\d+(?:\.\d+)?(?:e[+-]?\d+)?)', line, re.IGNORECASE)
        if reynolds_match:
            if current_reynolds is not None and data_lines:
                ca_data[current_reynolds] = data_lines
            # Extract Reynolds value from header
            current_reynolds = float(reynolds_match.group(1))
            data_lines = []
        else:
            # Parse data line
            try:
                values = [float(x) for x in line.split()]
                if values:
                    data_lines.append(values)
            except ValueError:
                pass
    
    # Don't forget the last block
    if current_reynolds is not None and data_lines:
        ca_data[current_reynolds] = data_lines
    
    return alpha_values, mach_values, reynolds_values, ca_data


def parse_pppcg_file(filepath):
    """Parse R273XCv2Pppcg.dat file format.
    
    Contains multiple coefficient tables: CN-A, CNd, Cma, CMd, CNq, CMq, CLLP, Cad
    Each table is indexed by alpha (rows) and mach (columns)
    """
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Extract alpha and mach from header
    # Support various formats: "alpha (rows):", "alpha(rows) :", "alpha(rows):", etc.
    alpha_match = re.search(r'alpha\s*\(rows\)\s*:\s*([\d.,\s]+)', content, re.IGNORECASE)
    # Support "mach (columns):", "mach(colomns) :", etc. (including typo "colomns")
    mach_match = re.search(r'mach\s*\(col[ou]m[n]?s\)\s*:\s*([\d.,\s]+)', content, re.IGNORECASE)
    
    # Also support "Alpha:" and "Mach:" format (simple colon format)
    # دعم تنسيق "Alpha:" و "Mach:" (تنسيق النقطتين البسيط)
    if not alpha_match:
        alpha_match = re.search(r'^alpha\s*:\s*([\d.,\s]+)', content, re.IGNORECASE | re.MULTILINE)
    if not mach_match:
        mach_match = re.search(r'^mach\s*:\s*([\d.,\s]+)', content, re.IGNORECASE | re.MULTILINE)
    
    # If still not found, try to infer from data structure (CMA/CNA blocks)
    # إذا لم يتم العثور عليه، حاول الاستدلال من بنية البيانات (كتل CMA/CNA)
    if not alpha_match or not mach_match:
        # Try to infer from first data block (CMA)
        # محاولة الاستدلال من أول كتلة بيانات (CMA)
        cma_match = re.search(r'^\s*CMA\s*$', content, re.MULTILINE)
        if cma_match:
            # Count rows in first data block (CMA) - these are alpha values
            # عد الصفوف في أول كتلة بيانات (CMA) - هذه هي قيم alpha
            lines = content.split('\n')
            cma_start_idx = None
            for i, line in enumerate(lines):
                if re.match(r'^\s*CMA\s*$', line):
                    cma_start_idx = i + 1
                    break
            
            if cma_start_idx:
                # Find first data row after CMA header
                # العثور على أول صف بيانات بعد رأس CMA
                data_start_idx = None
                for i in range(cma_start_idx, len(lines)):
                    line = lines[i].strip()
                    if line and not line.startswith('C') and not line.startswith('A'):
                        # Try to parse as numbers
                        try:
                            values = [float(x) for x in line.split()]
                            if len(values) > 0:
                                data_start_idx = i
                                break
                        except ValueError:
                            pass
                
                if data_start_idx:
                    # Count number of data rows (alpha values)
                    # عد عدد صفوف البيانات (قيم alpha)
                    num_alpha = 0
                    for i in range(data_start_idx, len(lines)):
                        line = lines[i].strip()
                        if not line:
                            break
                        try:
                            values = [float(x) for x in line.split()]
                            if len(values) > 0:
                                num_alpha += 1
                            else:
                                break
                        except ValueError:
                            break
                    
                    # Standard alpha values: 0, 4, 8, 12, 16, 20
                    # قيم alpha القياسية: 0, 4, 8, 12, 16, 20
                    if num_alpha == 6:
                        alpha_values = [0.0, 4.0, 8.0, 12.0, 16.0, 20.0]
                    else:
                        # Generate default alpha values
                        # إنشاء قيم alpha افتراضية
                        alpha_values = [float(i * 4) for i in range(num_alpha)]
                    
                    # Count number of columns in first data row (mach values)
                    # عد عدد الأعمدة في أول صف بيانات (قيم mach)
                    first_data_line = lines[data_start_idx].strip()
                    try:
                        values = [float(x) for x in first_data_line.split()]
                        num_mach = len(values)
                        # Standard mach values: 3, 4, 5, 6, 7, 8
                        # قيم mach القياسية: 3, 4, 5, 6, 7, 8
                        if num_mach == 6:
                            mach_values = [3.0, 4.0, 5.0, 6.0, 7.0, 8.0]
                        else:
                            # Generate default mach values
                            # إنشاء قيم mach افتراضية
                            mach_values = [float(i + 3) for i in range(num_mach)]
                    except ValueError:
                        raise ValueError(f"Could not parse mach values from {filepath}")
                else:
                    raise ValueError(f"Could not find alpha/mach headers in {filepath}")
            else:
                raise ValueError(f"Could not find alpha/mach headers in {filepath}")
        else:
            raise ValueError(f"Could not find alpha/mach headers in {filepath}")
    else:
        alpha_values = [float(x.strip()) for x in alpha_match.group(1).split(',')]
        mach_values = [float(x.strip()) for x in mach_match.group(1).split(',')]
    
    # Define coefficient names to extract
    # Support both standard format (CN-A, CNd, etc.) and simplified format (CMA, CNA, etc.)
    # دعم كل من التنسيق القياسي (CN-A, CNd, إلخ) والتنسيق المبسط (CMA, CNA, إلخ)
    coeff_names = ['CN-A', 'CNd', 'Cma', 'CMd', 'CNq', 'CMq', 'CLLP', 'Cad',
                   'CMA', 'CNA', 'CMQA', 'CNQA', 'CMD', 'CND', 'CLLPA', 'CAD']
    coefficients = {}
    
    lines = content.split('\n')
    current_coeff = None
    data_lines = []
    
    for line in lines:
        line_stripped = line.strip()
        
        # Check if this is a coefficient header
        for name in coeff_names:
            if line_stripped == name:
                if current_coeff is not None and data_lines:
                    coefficients[current_coeff] = np.array(data_lines)
                current_coeff = name
                data_lines = []
                break
        else:
            # Try to parse as data line
            if current_coeff and line_stripped:
                try:
                    values = [float(x) for x in line_stripped.split()]
                    if values:
                        data_lines.append(values)
                except ValueError:
                    pass
    
    # Don't forget the last coefficient
    if current_coeff is not None and data_lines:
        coefficients[current_coeff] = np.array(data_lines)
    
    return alpha_values, mach_values, coefficients


def parse_rr_file(filepath):
    """Parse R273XCv2RR.dat file format.
    
    Contains CLL (roll moment) coefficients at different roll deflections (0, 5, 10, 15 deg)
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    # Parse header
    header = lines[0].strip().split()
    num_alpha = int(header[0])
    num_mach = int(header[1])
    num_roll = int(header[2])
    
    # Parse alpha values
    alpha_values = [float(x) for x in lines[1].strip().split()]
    
    # Parse mach values
    mach_values = [float(x) for x in lines[2].strip().split()]
    
    # Parse roll deflection values
    roll_values = [float(x) for x in lines[3].strip().split()]
    
    # Parse CLL data blocks
    cll_data = {}
    current_roll = None
    data_lines = []
    
    for line in lines[4:]:
        line = line.strip()
        if not line:
            continue
        
        # Check if this is a roll header (e.g., "CLL0", "CLL5", "CLL10", "CLL15")
        if line.startswith('CLL'):
            if current_roll is not None and data_lines:
                cll_data[current_roll] = data_lines
            # Extract roll value from header
            roll_str = line[3:]  # Remove "CLL" prefix
            current_roll = float(roll_str)
            data_lines = []
        else:
            # Parse data line
            try:
                values = [float(x) for x in line.split()]
                if values:
                    data_lines.append(values)
            except ValueError:
                pass
    
    # Don't forget the last block
    if current_roll is not None and data_lines:
        cll_data[current_roll] = data_lines
    
    return alpha_values, mach_values, roll_values, cll_data


def find_datcom_input_file(datcom_output_path):
    """Find the corresponding DATCOM input file (for005.dat) for a DATCOM output file.
    
    The input file contains the alpha and mach values used in the simulation.
    
    Args:
        datcom_output_path: Path to the DATCOM output file (e.g., AERO. COEFF.DAT)
        
    Returns:
        str: Path to the input file, or None if not found
    """
    directory = os.path.dirname(datcom_output_path)
    
    # Common DATCOM input file names
    input_names = ['for005.dat', 'FOR005.DAT', 'for005.DAT', 'FOR005.dat']
    
    for name in input_names:
        input_path = os.path.join(directory, name)
        if os.path.exists(input_path):
            return input_path
    
    return None


def parse_datcom_input_file(filepath):
    """Parse DATCOM input file to extract alpha and mach values.
    
    Args:
        filepath: Path to the DATCOM input file (for005.dat)
        
    Returns:
        tuple: (alpha_values, mach_values) lists
    """
    with open(filepath, 'r') as f:
        content = f.read()
    
    mach_match = re.search(r'(?<![N])MACH\s*=\s*([\d.,\s\n]+?)(?:,\s*(?:NALPHA|REN|ALPHA|\$)|$)', content, re.IGNORECASE)
    if mach_match:
        mach_str = mach_match.group(1)
        mach_str = re.sub(r'\s+', ' ', mach_str.strip())
        mach_values = []
        for x in mach_str.split(','):
            x = x.strip()
            if x:
                try:
                    mach_values.append(float(x))
                except ValueError:
                    pass
    else:
        mach_values = []
    
    alpha_match = re.search(r'ALPHA\s*=\s*([\d.,\s\n-]+?)(?:,\s*\$|$)', content, re.IGNORECASE)
    if alpha_match:
        alpha_str = alpha_match.group(1)
        alpha_str = re.sub(r'\s+', ' ', alpha_str.strip())
        alpha_values = []
        for x in alpha_str.split(','):
            x = x.strip()
            if x:
                try:
                    alpha_values.append(float(x))
                except ValueError:
                    pass
    else:
        alpha_values = []
    
    return alpha_values, mach_values


def parse_datcom_cases(filepath):
    """Parse MISSILE DATCOM output file into multiple cases.
    
    The file contains multiple cases separated by "No of BETA is: N" headers.
    Each case has coefficient blocks (CN, CM, CA, etc.) with data organized
    as rows (alpha) x columns (mach).
    
    Args:
        filepath: Path to the DATCOM output file (e.g., AERO. COEFF.DAT)
        
    Returns:
        list: List of dicts, each containing:
            - 'case_num': Case number (1-based)
            - 'deflection': Deflection values from $DEFLCT line (if present)
            - 'coefficients': Dict of coefficient name -> numpy array
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    coeff_names = ['CN', 'CM', 'CA', 'CY', 'CLN', 'CLL', 'CL', 'CD', 'CNA', 'CMA', 
                   'CYB', 'CLNB', 'CLLB', 'CNQ', 'CMQ', 'CAQ', 'CNAD', 'CMAD',
                   'CYR', 'CLNR', 'CLLR', 'CYP', 'CLNP', 'CLLP']
    
    cases = []
    current_case = None
    current_coeff = None
    current_deflection = None
    data_lines = []
    
    for line in lines:
        line_stripped = line.strip()
        
        if re.match(r'No\s+of\s+BETA\s+is\s*:\s*\d+', line_stripped, re.IGNORECASE):
            if current_case is not None:
                if current_coeff is not None and data_lines:
                    current_case['coefficients'][current_coeff] = np.array(data_lines)
                cases.append(current_case)
            
            case_num = int(re.search(r'\d+', line_stripped).group())
            current_case = {
                'case_num': case_num,
                'deflection': None,
                'coefficients': {}
            }
            current_coeff = None
            current_deflection = None
            data_lines = []
            continue
        
        if current_case is None:
            continue
        
        if line_stripped.startswith('$DEFLCT'):
            deflct_match = re.search(r'DELTA1\s*=\s*([\d.,\s-]+)', line_stripped)
            if deflct_match:
                deflct_str = deflct_match.group(1)
                deflct_values = []
                for x in deflct_str.split(','):
                    x = x.strip()
                    if x:
                        try:
                            deflct_values.append(float(x))
                        except ValueError:
                            pass
                if deflct_values:
                    current_deflection = deflct_values[0]
                    if current_case['deflection'] is None:
                        current_case['deflection'] = current_deflection
            continue
        
        if line_stripped in coeff_names:
            if current_coeff is not None and data_lines:
                current_case['coefficients'][current_coeff] = np.array(data_lines)
            current_coeff = line_stripped
            data_lines = []
            continue
        
        if current_coeff and line_stripped:
            try:
                values = [float(x) for x in line_stripped.split()]
                if values:
                    data_lines.append(values)
            except ValueError:
                pass
    
    if current_case is not None:
        if current_coeff is not None and data_lines:
            current_case['coefficients'][current_coeff] = np.array(data_lines)
        cases.append(current_case)
    
    return cases


def parse_datcom_deflection_blocks(filepath):
    """Parse MISSILE DATCOM output file with multiple deflection blocks.
    
    Some DATCOM files have multiple deflection blocks within a single case,
    where each coefficient (CN, CM, CA, CLL) appears multiple times with
    different $DEFLCT values.
    
    Args:
        filepath: Path to the DATCOM output file (e.g., AERO. COEFF.DAT)
        
    Returns:
        dict: Dictionary mapping deflection value to dict of coefficients
              {deflection: {'CN': array, 'CM': array, 'CA': array, 'CLL': array, ...}}
    """
    with open(filepath, 'r') as f:
        lines = f.readlines()
    
    coeff_names = ['CN', 'CM', 'CA', 'CY', 'CLN', 'CLL', 'CL', 'CD',
                   'CNA', 'CMA', 'CYB', 'CLNB', 'CLLB',
                   'CNQ', 'CMQ', 'CAQ', 'CNAD', 'CMAD',
                   'CYR', 'CLNR', 'CLLR', 'CYP', 'CLNP', 'CLLP']
    
    deflection_blocks = {}
    current_deflection = 0.0
    current_coeff = None
    data_lines = []
    
    for line in lines:
        line_stripped = line.strip()
        
        if line_stripped.startswith('$DEFLCT'):
            if current_coeff is not None and data_lines:
                if current_deflection not in deflection_blocks:
                    deflection_blocks[current_deflection] = {}
                deflection_blocks[current_deflection][current_coeff] = np.array(data_lines)
                current_coeff = None
                data_lines = []
            
            deflct_match = re.search(r'DELTA1\s*=\s*([\d.,\s-]+)', line_stripped)
            if deflct_match:
                deflct_str = deflct_match.group(1)
                deflct_values = []
                for x in deflct_str.split(','):
                    x = x.strip()
                    if x:
                        try:
                            deflct_values.append(float(x))
                        except ValueError:
                            pass
                if deflct_values:
                    current_deflection = deflct_values[0]
            continue
        
        if line_stripped in coeff_names:
            if current_coeff is not None and data_lines:
                if current_deflection not in deflection_blocks:
                    deflection_blocks[current_deflection] = {}
                deflection_blocks[current_deflection][current_coeff] = np.array(data_lines)
            current_coeff = line_stripped
            data_lines = []
            continue
        
        if current_coeff and line_stripped:
            try:
                values = [float(x) for x in line_stripped.split()]
                if values:
                    data_lines.append(values)
            except ValueError:
                pass
    
    if current_coeff is not None and data_lines:
        if current_deflection not in deflection_blocks:
            deflection_blocks[current_deflection] = {}
        deflection_blocks[current_deflection][current_coeff] = np.array(data_lines)
    
    return deflection_blocks


def parse_datcom_reynolds_cases(filepath):
    """Parse DATCOM input file to extract Reynolds values for each case.
    
    The file may have multiple $FLTCON blocks separated by NEXT CASE.
    Each block has REN=N*value format (e.g., REN=8*1.0E+05).
    
    Args:
        filepath: Path to the DATCOM input file (for005.dat)
        
    Returns:
        list: List of Reynolds values, one per case
    """
    with open(filepath, 'r') as f:
        content = f.read()
    
    reynolds_values = []
    
    ren_matches = re.findall(r'REN\s*=\s*(\d+)\*([0-9.eE+\-]+)', content, re.IGNORECASE)
    for match in ren_matches:
        try:
            reynolds_values.append(float(match[1]))
        except ValueError:
            pass
    
    if not reynolds_values:
        ren_match = re.search(r'REN\s*=\s*([\d.,eE+\-\s\n]+?)(?:,\s*(?:NALPHA|ALPHA|\$)|$)', content, re.IGNORECASE)
        if ren_match:
            ren_str = ren_match.group(1)
            ren_str = re.sub(r'\s+', ' ', ren_str.strip())
            for x in ren_str.split(','):
                x = x.strip()
                if x:
                    try:
                        reynolds_values.append(float(x))
                    except ValueError:
                        pass
    
    return reynolds_values


def parse_datcom_deflection_cases(filepath):
    """Parse DATCOM input file to extract deflection values for each case.
    
    The file may have multiple $DEFLCT lines, each followed by NEXT CASE.
    
    Args:
        filepath: Path to the DATCOM input file (for005.dat)
        
    Returns:
        list: List of deflection values, one per case
    """
    with open(filepath, 'r') as f:
        content = f.read()
    
    deflection_values = []
    
    deflct_matches = re.findall(r'\$DEFLCT\s+DELTA1\s*=\s*([\d.,\s-]+)', content, re.IGNORECASE)
    for match in deflct_matches:
        values = []
        for x in match.split(','):
            x = x.strip()
            if x:
                try:
                    values.append(float(x))
                except ValueError:
                    pass
        if values:
            deflection_values.append(values[0])
    
    return deflection_values


def parse_datcom_file(filepath):
    """Parse MISSILE DATCOM output file format (legacy interface).
    
    Returns the first case's coefficients for backward compatibility.
    """
    input_file = find_datcom_input_file(filepath)
    if input_file:
        alpha_values, mach_values = parse_datcom_input_file(input_file)
    else:
        alpha_values, mach_values = [], []
    
    cases = parse_datcom_cases(filepath)
    
    if cases:
        coefficients = cases[0]['coefficients']
    else:
        coefficients = {}
    
    if not alpha_values and coefficients:
        first_coeff = list(coefficients.values())[0]
        alpha_values = list(range(first_coeff.shape[0]))
    if not mach_values and coefficients:
        first_coeff = list(coefficients.values())[0]
        mach_values = list(range(first_coeff.shape[1]))
    
    return alpha_values, mach_values, coefficients


def create_datcom_aero_csv(alpha_values, mach_values, coefficients, output_path):
    """Create aero_coeffs.csv from DATCOM output coefficients.
    
    Maps:
    - Cd <- CD (drag coefficient, or CA if CD not available)
    - Cn <- CN (normal force coefficient)
    - Cm <- CM (pitching moment coefficient)
    """
    rows = []
    
    cn_data = coefficients.get('CN', np.zeros((len(alpha_values), len(mach_values))))
    cm_data = coefficients.get('CM', np.zeros((len(alpha_values), len(mach_values))))
    cd_data = coefficients.get('CD', coefficients.get('CA', np.zeros((len(alpha_values), len(mach_values)))))
    
    actual_mach_count = cn_data.shape[1] if len(cn_data.shape) > 1 else len(mach_values)
    if len(mach_values) > actual_mach_count:
        print(f"    Warning: Mach list ({len(mach_values)}) exceeds data columns ({actual_mach_count}), trimming to match data")
        mach_values = mach_values[:actual_mach_count]
    
    for j, mach in enumerate(mach_values):
        for i in range(len(alpha_values) - 1, 0, -1):
            alpha = alpha_values[i]
            if alpha > 0:
                cn = -cn_data[i][j] if i < len(cn_data) and j < len(cn_data[i]) else 0
                cm = -cm_data[i][j] if i < len(cm_data) and j < len(cm_data[i]) else 0
                cd = cd_data[i][j] if i < len(cd_data) and j < len(cd_data[i]) else 0
                rows.append({
                    'Mach': mach,
                    'alpha_deg': -alpha,
                    'Cd': cd,
                    'Cn': cn,
                    'Cm': -cm
                })
        
        for i, alpha in enumerate(alpha_values):
            cn = cn_data[i][j] if i < len(cn_data) and j < len(cn_data[i]) else 0
            cm = cm_data[i][j] if i < len(cm_data) and j < len(cm_data[i]) else 0
            cd = cd_data[i][j] if i < len(cd_data) and j < len(cd_data[i]) else 0
            rows.append({
                'Mach': mach,
                'alpha_deg': alpha,
                'Cd': cd,
                'Cn': cn,
                'Cm': cm
            })
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['Mach', 'alpha_deg'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_datcom_ca_3d_csv(alpha_values, mach_values, cases, reynolds_values, output_path):
    """Create ca_3d_coeffs.csv from DATCOM output with Reynolds dimension.
    
    Each case corresponds to a different Reynolds number.
    """
    rows = []
    
    for case_idx, case in enumerate(cases):
        if case_idx >= len(reynolds_values):
            break
        reynolds = reynolds_values[case_idx]
        ca_data = case['coefficients'].get('CA', None)
        if ca_data is None:
            continue
        
        for i, alpha in enumerate(alpha_values):
            if i >= len(ca_data):
                continue
            for j, mach in enumerate(mach_values):
                if j >= len(ca_data[i]):
                    continue
                rows.append({
                    'Mach': mach,
                    'alpha_deg': alpha,
                    'Reynolds': reynolds,
                    'CA': ca_data[i][j]
                })
    
    if not rows:
        print(f"Warning: No CA data found for {output_path}")
        return None
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['Mach', 'alpha_deg', 'Reynolds'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_datcom_damping_csv(alpha_values, mach_values, coefficients, output_path):
    """Create damping_coeffs.csv from DATCOM output.
    
    Maps:
    - Cmq <- CMQ (pitch damping)
    - Cnq <- CNQ (normal force due to pitch rate)
    - Clp <- CLLP (roll damping)
    """
    rows = []
    
    cmq_data = coefficients.get('CMQ', np.zeros((len(alpha_values), len(mach_values))))
    cnq_data = coefficients.get('CNQ', np.zeros((len(alpha_values), len(mach_values))))
    clp_data = coefficients.get('CLLP', np.zeros((len(alpha_values), len(mach_values))))
    
    has_data = ('CMQ' in coefficients or 'CNQ' in coefficients or 'CLLP' in coefficients)
    if not has_data:
        print(f"Warning: No damping coefficients found for {output_path}")
        return None
    
    actual_mach_count = cmq_data.shape[1] if len(cmq_data.shape) > 1 else len(mach_values)
    if len(mach_values) > actual_mach_count:
        print(f"    Warning: Mach list ({len(mach_values)}) exceeds data columns ({actual_mach_count}), trimming to match data")
        mach_values = mach_values[:actual_mach_count]
    
    for j, mach in enumerate(mach_values):
        for i in range(len(alpha_values) - 1, 0, -1):
            alpha = alpha_values[i]
            if alpha > 0:
                cmq = cmq_data[i][j] if i < len(cmq_data) and j < len(cmq_data[i]) else 0
                cnq = cnq_data[i][j] if i < len(cnq_data) and j < len(cnq_data[i]) else 0
                clp = clp_data[i][j] if i < len(clp_data) and j < len(clp_data[i]) else 0
                rows.append({
                    'Mach': mach,
                    'alpha_deg': -alpha,
                    'Cmq': cmq,
                    'Cnq': cnq,
                    'Clp': clp
                })
        
        for i, alpha in enumerate(alpha_values):
            cmq = cmq_data[i][j] if i < len(cmq_data) and j < len(cmq_data[i]) else 0
            cnq = cnq_data[i][j] if i < len(cnq_data) and j < len(cnq_data[i]) else 0
            clp = clp_data[i][j] if i < len(clp_data) and j < len(clp_data[i]) else 0
            rows.append({
                'Mach': mach,
                'alpha_deg': alpha,
                'Cmq': cmq,
                'Cnq': cnq,
                'Clp': clp
            })
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['Mach', 'alpha_deg'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_datcom_roll_csv_from_blocks(alpha_values, mach_values, deflection_blocks, output_path):
    """Create roll_aero_coeffs.csv from DATCOM deflection blocks.
    
    deflection_blocks is a dict mapping deflection value to coefficients.
    Includes all deflection values including 0.
    """
    rows = []
    
    for deflection, coeffs in sorted(deflection_blocks.items()):
        cll_data = coeffs.get('CLL', None)
        if cll_data is None:
            continue
        
        for j, mach in enumerate(mach_values):
            for i, alpha in enumerate(alpha_values):
                if i >= len(cll_data) or j >= len(cll_data[i]):
                    continue
                rows.append({
                    'Mach': mach,
                    'alpha_deg': alpha,
                    'def_roll': deflection,
                    'Cll': cll_data[i][j]
                })
    
    if not rows:
        print(f"Warning: No roll deflection data found for {output_path}")
        return None
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['def_roll', 'Mach', 'alpha_deg'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_datcom_roll_csv(alpha_values, mach_values, cases, deflection_values, output_path):
    """Create roll_aero_coeffs.csv from DATCOM output with roll deflection.
    
    Each case corresponds to a different roll deflection angle.
    deflection_values comes from parsing the input file (for005.dat).
    Includes all deflection values including 0.
    """
    rows = []
    
    for case_idx, case in enumerate(cases):
        if case_idx < len(deflection_values):
            deflection = deflection_values[case_idx]
        else:
            deflection = case.get('deflection', 0)
        
        cll_data = case['coefficients'].get('CLL', None)
        if cll_data is None:
            continue
        
        for j, mach in enumerate(mach_values):
            for i, alpha in enumerate(alpha_values):
                if i >= len(cll_data) or j >= len(cll_data[i]):
                    continue
                rows.append({
                    'Mach': mach,
                    'alpha_deg': alpha,
                    'def_roll': deflection,
                    'Cll': cll_data[i][j]
                })
    
    if not rows:
        print(f"Warning: No roll deflection data found for {output_path}")
        return None
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['def_roll', 'Mach', 'alpha_deg'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_datcom_fin_deflection_csv_from_blocks(alpha_values, mach_values, deflection_blocks, output_path):
    """Create fin_deflection_coeffs.csv from DATCOM deflection blocks.
    
    Computes CNd and CMd by finite difference between deflection cases.
    CNd = (CN_deflected - CN_zero) / deflection_rad  (per radian)
    CMd = (CM_deflected - CM_zero) / deflection_rad  (per radian)
    
    Note: Output units are per-radian as expected by the 6DOF simulation code.
    DATCOM deflection values are in degrees, so they are converted to radians
    before computing the derivatives.
    
    deflection_blocks is a dict mapping deflection value (in degrees) to coefficients.
    Includes all deflection values including 0 (with Cnd=0, Cmd=0 for deflection=0).
    """
    if 0.0 not in deflection_blocks:
        print(f"Warning: No zero deflection case found for {output_path}")
        return None
    
    zero_coeffs = deflection_blocks[0.0]
    cn_zero = zero_coeffs.get('CN', None)
    cm_zero = zero_coeffs.get('CM', None)
    
    if cn_zero is None or cm_zero is None:
        print(f"Warning: Missing CN or CM in zero deflection case for {output_path}")
        return None
    
    actual_mach_count = cn_zero.shape[1] if len(cn_zero.shape) > 1 else len(mach_values)
    if len(mach_values) > actual_mach_count:
        print(f"    Warning: Mach list ({len(mach_values)}) exceeds data columns ({actual_mach_count}), trimming to match data")
        mach_values = mach_values[:actual_mach_count]
    
    rows = []
    
    for deflection, coeffs in sorted(deflection_blocks.items()):
        cn_data = coeffs.get('CN', None)
        cm_data = coeffs.get('CM', None)
        
        if cn_data is None or cm_data is None:
            continue
        
        for j, mach in enumerate(mach_values):
            for i, alpha in enumerate(alpha_values):
                if i >= len(cn_data) or j >= len(cn_data[i]):
                    continue
                if i >= len(cn_zero) or j >= len(cn_zero[i]):
                    continue
                
                if deflection == 0:
                    cnd = 0.0
                    cmd = 0.0
                else:
                    deflection_rad = np.radians(deflection)
                    cnd = (cn_data[i][j] - cn_zero[i][j]) / deflection_rad
                    cmd = (cm_data[i][j] - cm_zero[i][j]) / deflection_rad
                
                rows.append({
                    'Mach': mach,
                    'alpha_deg': alpha,
                    'def_pitch': deflection,
                    'Cnd': cnd,
                    'Cmd': cmd
                })
    
    if not rows:
        print(f"Warning: No fin deflection data found for {output_path}")
        return None
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['def_pitch', 'Mach', 'alpha_deg'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_datcom_fin_deflection_csv(alpha_values, mach_values, cases, deflection_values, output_path):
    """Create fin_deflection_coeffs.csv from DATCOM output.
    
    Computes CNd and CMd by finite difference between deflection cases.
    CNd = (CN_deflected - CN_zero) / deflection
    CMd = (CM_deflected - CM_zero) / deflection
    
    deflection_values comes from parsing the input file (for005.dat).
    """
    zero_case = None
    deflected_cases = []
    
    for case_idx, case in enumerate(cases):
        if case_idx < len(deflection_values):
            deflection = deflection_values[case_idx]
        else:
            deflection = case.get('deflection', 0)
        
        case['deflection'] = deflection
        
        if deflection == 0:
            zero_case = case
        else:
            deflected_cases.append(case)
    
    if zero_case is None or not deflected_cases:
        print(f"Warning: Need both zero and deflected cases for {output_path}")
        return None
    
    cn_zero = zero_case['coefficients'].get('CN', None)
    cm_zero = zero_case['coefficients'].get('CM', None)
    
    if cn_zero is None or cm_zero is None:
        print(f"Warning: Missing CN/CM in zero deflection case for {output_path}")
        return None
    
    rows = []
    
    for case in deflected_cases:
        deflection = case.get('deflection', 0)
        if deflection == 0:
            continue
        
        cn_defl = case['coefficients'].get('CN', None)
        cm_defl = case['coefficients'].get('CM', None)
        
        if cn_defl is None or cm_defl is None:
            continue
        
        for j, mach in enumerate(mach_values):
            for i in range(len(alpha_values) - 1, 0, -1):
                alpha = alpha_values[i]
                if alpha > 0:
                    if i < len(cn_zero) and j < len(cn_zero[i]) and i < len(cn_defl) and j < len(cn_defl[i]):
                        cnd = (cn_defl[i][j] - cn_zero[i][j]) / deflection
                        cmd = (cm_defl[i][j] - cm_zero[i][j]) / deflection
                        rows.append({
                            'Mach': mach,
                            'alpha_deg': -alpha,
                            'CNd': abs(cnd),
                            'CMd': abs(cmd)
                        })
            
            for i, alpha in enumerate(alpha_values):
                if i < len(cn_zero) and j < len(cn_zero[i]) and i < len(cn_defl) and j < len(cn_defl[i]):
                    cnd = (cn_defl[i][j] - cn_zero[i][j]) / deflection
                    cmd = (cm_defl[i][j] - cm_zero[i][j]) / deflection
                    rows.append({
                        'Mach': mach,
                        'alpha_deg': alpha,
                        'CNd': abs(cnd),
                        'CMd': abs(cmd)
                    })
        break
    
    if not rows:
        print(f"Warning: Could not compute fin deflection derivatives for {output_path}")
        return None
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['Mach', 'alpha_deg'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_ca_3d_csv(alpha_values, mach_values, reynolds_values, ca_data, output_path):
    """Create ca_3d_coeffs.csv from CA data."""
    rows = []
    
    for reynolds in sorted(ca_data.keys()):
        data_matrix = ca_data[reynolds]
        for i, alpha in enumerate(alpha_values):
            if i < len(data_matrix):
                for j, mach in enumerate(mach_values):
                    if j < len(data_matrix[i]):
                        rows.append({
                            'Mach': mach,
                            'alpha_deg': alpha,
                            'Reynolds': reynolds,
                            'CA': data_matrix[i][j]
                        })
    
    df = pd.DataFrame(rows)
    # Sort by Mach, alpha, Reynolds
    df = df.sort_values(['Mach', 'alpha_deg', 'Reynolds'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_aero_coeffs_csv(alpha_values, mach_values, coefficients, output_path):
    """Create aero_coeffs.csv from CN-A, Cma, Cad coefficients.
    
    Maps:
    - Cd <- Cad (axial drag coefficient from deflection)
    - Cn <- CN-A (normal force coefficient)
    - Cm <- Cma (pitching moment coefficient)
    """
    rows = []
    
    cn_data = coefficients.get('CN-A', np.zeros((len(alpha_values), len(mach_values))))
    cm_data = coefficients.get('Cma', np.zeros((len(alpha_values), len(mach_values))))
    cd_data = coefficients.get('Cad', np.zeros((len(alpha_values), len(mach_values))))
    
    for j, mach in enumerate(mach_values):
        # Add negative alpha values (symmetric)
        for i in range(len(alpha_values) - 1, 0, -1):
            alpha = alpha_values[i]
            if alpha > 0:
                cn = -cn_data[i][j] if i < len(cn_data) and j < len(cn_data[i]) else 0
                cm = -cm_data[i][j] if i < len(cm_data) and j < len(cm_data[i]) else 0
                cd = cd_data[i][j] if i < len(cd_data) and j < len(cd_data[i]) else 0
                rows.append({
                    'Mach': mach,
                    'alpha_deg': -alpha,
                    'Cd': cd,
                    'Cn': cn,
                    'Cm': -cm  # Cm sign convention
                })
        
        # Add positive alpha values
        for i, alpha in enumerate(alpha_values):
            cn = cn_data[i][j] if i < len(cn_data) and j < len(cn_data[i]) else 0
            cm = cm_data[i][j] if i < len(cm_data) and j < len(cm_data[i]) else 0
            cd = cd_data[i][j] if i < len(cd_data) and j < len(cd_data[i]) else 0
            rows.append({
                'Mach': mach,
                'alpha_deg': alpha,
                'Cd': cd,
                'Cn': cn,
                'Cm': cm
            })
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['Mach', 'alpha_deg'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_damping_coeffs_csv(alpha_values, mach_values, coefficients, output_path):
    """Create damping_coeffs.csv from CMq, CNq, CLLP coefficients."""
    rows = []
    
    cmq_data = coefficients.get('CMq', np.zeros((len(alpha_values), len(mach_values))))
    cnq_data = coefficients.get('CNq', np.zeros((len(alpha_values), len(mach_values))))
    clp_data = coefficients.get('CLLP', np.zeros((len(alpha_values), len(mach_values))))
    
    for j, mach in enumerate(mach_values):
        # Add negative alpha values (symmetric)
        for i in range(len(alpha_values) - 1, 0, -1):
            alpha = alpha_values[i]
            if alpha > 0:
                cmq = cmq_data[i][j] if i < len(cmq_data) and j < len(cmq_data[i]) else 0
                cnq = cnq_data[i][j] if i < len(cnq_data) and j < len(cnq_data[i]) else 0
                clp = clp_data[i][j] if i < len(clp_data) and j < len(clp_data[i]) else 0
                rows.append({
                    'Mach': mach,
                    'alpha_deg': -alpha,
                    'Cmq': cmq,
                    'Cnq': cnq,
                    'Clp': clp
                })
        
        # Add positive alpha values
        for i, alpha in enumerate(alpha_values):
            cmq = cmq_data[i][j] if i < len(cmq_data) and j < len(cmq_data[i]) else 0
            cnq = cnq_data[i][j] if i < len(cnq_data) and j < len(cnq_data[i]) else 0
            clp = clp_data[i][j] if i < len(clp_data) and j < len(clp_data[i]) else 0
            rows.append({
                'Mach': mach,
                'alpha_deg': alpha,
                'Cmq': cmq,
                'Cnq': cnq,
                'Clp': clp
            })
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['Mach', 'alpha_deg'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_roll_aero_coeffs_csv(alpha_values, mach_values, roll_values, cll_data, output_path):
    """Create roll_aero_coeffs.csv from CLL data."""
    rows = []
    
    for roll_def in sorted(cll_data.keys()):
        if roll_def == 0:  # Skip zero deflection (all zeros)
            continue
        data_matrix = cll_data[roll_def]
        for j, mach in enumerate(mach_values):
            for i, alpha in enumerate(alpha_values):
                if i < len(data_matrix) and j < len(data_matrix[i]):
                    rows.append({
                        'Mach': mach,
                        'alpha_deg': alpha,
                        'def_roll': roll_def,
                        'Cll': data_matrix[i][j]
                    })
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['def_roll', 'Mach', 'alpha_deg'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def create_fin_deflection_coeffs_csv(alpha_values, mach_values, coefficients, output_path):
    """Create fin_deflection_coeffs.csv from CNd, CMd coefficients.
    
    CNd - Normal force derivative with respect to fin deflection
    CMd - Pitching moment derivative with respect to fin deflection
    """
    rows = []
    
    cnd_data = coefficients.get('CNd', np.zeros((len(alpha_values), len(mach_values))))
    cmd_data = coefficients.get('CMd', np.zeros((len(alpha_values), len(mach_values))))
    
    for j, mach in enumerate(mach_values):
        # Add negative alpha values (symmetric)
        for i in range(len(alpha_values) - 1, 0, -1):
            alpha = alpha_values[i]
            if alpha > 0:
                cnd = cnd_data[i][j] if i < len(cnd_data) and j < len(cnd_data[i]) else 0
                cmd = cmd_data[i][j] if i < len(cmd_data) and j < len(cmd_data[i]) else 0
                rows.append({
                    'Mach': mach,
                    'alpha_deg': -alpha,
                    'CNd': cnd,
                    'CMd': cmd
                })
        
        # Add positive alpha values
        for i, alpha in enumerate(alpha_values):
            cnd = cnd_data[i][j] if i < len(cnd_data) and j < len(cnd_data[i]) else 0
            cmd = cmd_data[i][j] if i < len(cmd_data) and j < len(cmd_data[i]) else 0
            rows.append({
                'Mach': mach,
                'alpha_deg': alpha,
                'CNd': cnd,
                'CMd': cmd
            })
    
    df = pd.DataFrame(rows)
    df = df.sort_values(['Mach', 'alpha_deg'])
    df.to_csv(output_path, index=False)
    print(f"Created {output_path} with {len(df)} rows")
    return df


def find_excel_files(directory):
    """Find all Excel files in a directory and its subdirectories.
    
    Args:
        directory: Path to search for Excel files
        
    Returns:
        list: List of Excel file paths (.xlsx, .xls)
    """
    excel_files = []
    
    for pattern in ['*.xlsx', '*.xls', '*.XLSX', '*.XLS']:
        excel_files.extend(glob.glob(os.path.join(directory, pattern)))
        for subdir in os.listdir(directory):
            subdir_path = os.path.join(directory, subdir)
            if os.path.isdir(subdir_path):
                excel_files.extend(glob.glob(os.path.join(subdir_path, pattern)))
    
    return sorted(set(excel_files))


def convert_excel_to_csv(excel_path, output_dir=None):
    """Convert an Excel file to CSV format.
    
    If the Excel file has multiple sheets, each sheet is saved as a separate CSV file.
    Single-sheet files are saved with the same name as the Excel file.
    Multi-sheet files are saved with the sheet name appended.
    
    Args:
        excel_path: Path to the Excel file
        output_dir: Directory to save CSV files (default: same as Excel file)
        
    Returns:
        list: List of created CSV file paths
    """
    if not EXCEL_SUPPORT:
        print(f"  Warning: openpyxl not installed, cannot convert {excel_path}")
        return []
    
    if output_dir is None:
        output_dir = os.path.dirname(excel_path)
    
    try:
        xls = pd.ExcelFile(excel_path)
        sheet_names = xls.sheet_names
        created_files = []
        
        base_name = os.path.splitext(os.path.basename(excel_path))[0]
        
        for sheet_name in sheet_names:
            try:
                df = pd.read_excel(xls, sheet_name=sheet_name)
                
                if df.empty:
                    continue
                
                if len(sheet_names) == 1:
                    csv_name = f"{base_name}.csv"
                else:
                    safe_sheet_name = re.sub(r'[^\w\s-]', '', sheet_name).strip()
                    safe_sheet_name = re.sub(r'[-\s]+', '_', safe_sheet_name)
                    csv_name = f"{base_name}_{safe_sheet_name}.csv"
                
                csv_path = os.path.join(output_dir, csv_name)
                df.to_csv(csv_path, index=False)
                created_files.append(csv_path)
                print(f"  Created {csv_path} ({len(df)} rows, {len(df.columns)} columns)")
                
            except Exception as e:
                print(f"  Warning: Could not convert sheet '{sheet_name}': {e}")
        
        return created_files
        
    except Exception as e:
        print(f"  Error reading Excel file {excel_path}: {e}")
        return []


def process_excel_files(directory, output_dir=None):
    """Find and convert all Excel files in a directory.
    
    Args:
        directory: Path to search for Excel files
        output_dir: Directory to save CSV files (default: same as each Excel file)
        
    Returns:
        dict: Summary with 'converted' count and 'errors' list
    """
    summary = {'converted': 0, 'errors': []}
    
    if not EXCEL_SUPPORT:
        print("  Warning: openpyxl not installed. Install with: pip install openpyxl")
        return summary
    
    excel_files = find_excel_files(directory)
    
    if not excel_files:
        return summary
    
    print(f"\n  Found {len(excel_files)} Excel file(s)")
    
    for excel_file in excel_files:
        rel_path = os.path.relpath(excel_file, directory)
        print(f"\n  Converting: {rel_path}")
        
        if output_dir:
            csv_output_dir = output_dir
        else:
            csv_output_dir = os.path.dirname(excel_file)
        
        created = convert_excel_to_csv(excel_file, csv_output_dir)
        
        if created:
            summary['converted'] += len(created)
        else:
            summary['errors'].append(f"Failed to convert {rel_path}")
    
    return summary


def find_sibling_directories(base_dir):
    """Find all sibling directories that might contain aerodynamic data.
    
    This function looks at the parent directory of base_dir and finds all
    sibling directories (excluding 'rocket_models' and files).
    
    Args:
        base_dir: The directory where the script is located
        
    Returns:
        list: List of directory paths that might contain aerodynamic data
    """
    parent_dir = os.path.dirname(base_dir)
    sibling_dirs = []
    
    # Directories to exclude from processing
    exclude_dirs = {'rocket_models', '__pycache__', '.git'}
    
    for item in os.listdir(parent_dir):
        item_path = os.path.join(parent_dir, item)
        if os.path.isdir(item_path) and item not in exclude_dirs:
            sibling_dirs.append(item_path)
    
    return sorted(sibling_dirs)


def has_aero_data(directory):
    """Check if a directory contains aerodynamic data files.
    
    Args:
        directory: Path to check
        
    Returns:
        bool: True if the directory contains aerodynamic data files
    """
    files = find_data_files(directory)
    return sum(len(f) for f in files.values()) > 0


def process_directory(directory, output_dir=None):
    """Process a single directory and convert aerodynamic data to CSV.
    
    Args:
        directory: Path to the directory containing aerodynamic data
        output_dir: Optional output directory (defaults to the input directory)
        
    Returns:
        dict: Summary of processed files
    """
    if output_dir is None:
        output_dir = directory
    
    summary = {'ca': 0, 'pppcg': 0, 'rr': 0, 'datcom': 0, 'errors': []}
    
    print(f"\nProcessing directory: {directory}")
    print("-" * 60)
    
    # Find and categorize all data files
    files = find_data_files(directory)
    
    total_files = sum(len(f) for f in files.values())
    if total_files == 0:
        print("  No aerodynamic data files found in this directory.")
        return summary
    
    print(f"  Found {total_files} data file(s):")
    print(f"    - CA files: {len(files['ca'])}")
    print(f"    - PPPCG files: {len(files['pppcg'])}")
    print(f"    - RR files: {len(files['rr'])}")
    print(f"    - DATCOM files: {len(files['datcom'])}")
    print()
    
    # Process CA files (3D axial force coefficients)
    for ca_file in sorted(files['ca']):
        try:
            alpha_ca, mach_ca, reynolds_ca, ca_data = parse_ca_file(ca_file)
            filename = os.path.basename(ca_file)
            print(f"  Parsed CA file: {filename}")
            print(f"    Alpha values: {alpha_ca}")
            print(f"    Mach values: {mach_ca}")
            print(f"    Reynolds values: {reynolds_ca}")
            print(f"    CA blocks: {list(ca_data.keys())}")
            
            # Generate output filename based on source file
            base_name = os.path.splitext(filename)[0]
            output_name = f'ca_3d_coeffs_{base_name}.csv'
            create_ca_3d_csv(alpha_ca, mach_ca, reynolds_ca, ca_data,
                           os.path.join(output_dir, output_name))
            summary['ca'] += 1
            print()
        except Exception as e:
            error_msg = f"Error processing CA file {ca_file}: {e}"
            print(f"  {error_msg}")
            summary['errors'].append(error_msg)
            print()
    
    # Process PPPCG files (2D coefficient tables)
    for pppcg_file in sorted(files['pppcg']):
        try:
            alpha_pp, mach_pp, coefficients = parse_pppcg_file(pppcg_file)
            filename = os.path.basename(pppcg_file)
            print(f"  Parsed PPPCG file: {filename}")
            print(f"    Alpha values: {alpha_pp}")
            print(f"    Mach values: {mach_pp}")
            print(f"    Coefficients: {list(coefficients.keys())}")
            
            # Create aero_coeffs.csv
            create_aero_coeffs_csv(alpha_pp, mach_pp, coefficients,
                                 os.path.join(output_dir, 'aero_coeffs.csv'))
            
            # Create damping_coeffs.csv
            create_damping_coeffs_csv(alpha_pp, mach_pp, coefficients,
                                    os.path.join(output_dir, 'damping_coeffs.csv'))
            
            # Create fin_deflection_coeffs.csv
            create_fin_deflection_coeffs_csv(alpha_pp, mach_pp, coefficients,
                                           os.path.join(output_dir, 'fin_deflection_coeffs.csv'))
            summary['pppcg'] += 1
            print()
        except Exception as e:
            error_msg = f"Error processing PPPCG file {pppcg_file}: {e}"
            print(f"  {error_msg}")
            summary['errors'].append(error_msg)
            print()
    
    # Process RR files (roll coefficients)
    for rr_file in sorted(files['rr']):
        try:
            alpha_rr, mach_rr, roll_rr, cll_data = parse_rr_file(rr_file)
            filename = os.path.basename(rr_file)
            print(f"  Parsed RR file: {filename}")
            print(f"    Alpha values: {alpha_rr}")
            print(f"    Mach values: {mach_rr}")
            print(f"    Roll values: {roll_rr}")
            print(f"    CLL blocks: {list(cll_data.keys())}")
            
            # Create roll_aero_coeffs.csv
            create_roll_aero_coeffs_csv(alpha_rr, mach_rr, roll_rr, cll_data,
                                      os.path.join(output_dir, 'roll_aero_coeffs.csv'))
            summary['rr'] += 1
            print()
        except Exception as e:
            error_msg = f"Error processing RR file {rr_file}: {e}"
            print(f"  {error_msg}")
            summary['errors'].append(error_msg)
            print()
    
    # Process DATCOM files (MISSILE DATCOM output)
    # Group DATCOM files by their purpose based on folder names
    datcom_ren_off = []
    datcom_ren_on = []
    datcom_roll = []
    datcom_delta = []
    datcom_other = []
    
    for datcom_file in files['datcom']:
        parent_dir = os.path.basename(os.path.dirname(datcom_file)).lower()
        if 'ren' in parent_dir and 'off' in parent_dir:
            datcom_ren_off.append(datcom_file)
        elif 'ren' in parent_dir and 'on' in parent_dir:
            datcom_ren_on.append(datcom_file)
        elif 'roll' in parent_dir:
            datcom_roll.append(datcom_file)
        elif 'delta' in parent_dir and 'roll' not in parent_dir:
            datcom_delta.append(datcom_file)
        else:
            datcom_other.append(datcom_file)
    
    # Process Reynolds-based DATCOM files (motor off) -> ca_3d_coeffs_motor_off.csv ONLY
    # Note: aero_coeffs.csv and damping_coeffs.csv are generated from pitch deflection folder
    for datcom_file in sorted(datcom_ren_off):
        try:
            input_file = find_datcom_input_file(datcom_file)
            alpha_dc, mach_dc = parse_datcom_input_file(input_file) if input_file else ([], [])
            reynolds_values = parse_datcom_reynolds_cases(input_file) if input_file else []
            cases = parse_datcom_cases(datcom_file)
            
            filename = os.path.basename(datcom_file)
            parent_dir = os.path.basename(os.path.dirname(datcom_file))
            print(f"  Parsed DATCOM file (Reynolds, motor off): {parent_dir}/{filename}")
            print(f"    Alpha values: {alpha_dc}")
            print(f"    Mach values: {mach_dc}")
            print(f"    Reynolds values: {reynolds_values}")
            print(f"    Cases found: {len(cases)}")
            
            if cases:
                first_case_coeffs = cases[0]['coefficients']
                print(f"    Coefficients in first case: {list(first_case_coeffs.keys())}")
                
                if reynolds_values:
                    create_datcom_ca_3d_csv(alpha_dc, mach_dc, cases, reynolds_values,
                                          os.path.join(output_dir, 'ca_3d_coeffs_motor_off.csv'))
            
            summary['datcom'] += 1
            print()
        except Exception as e:
            error_msg = f"Error processing DATCOM file {datcom_file}: {e}"
            print(f"  {error_msg}")
            summary['errors'].append(error_msg)
            print()
    
    # Process Reynolds-based DATCOM files (motor on) -> ca_3d_coeffs_motor_on.csv
    for datcom_file in sorted(datcom_ren_on):
        try:
            input_file = find_datcom_input_file(datcom_file)
            alpha_dc, mach_dc = parse_datcom_input_file(input_file) if input_file else ([], [])
            reynolds_values = parse_datcom_reynolds_cases(input_file) if input_file else []
            cases = parse_datcom_cases(datcom_file)
            
            filename = os.path.basename(datcom_file)
            parent_dir = os.path.basename(os.path.dirname(datcom_file))
            print(f"  Parsed DATCOM file (Reynolds, motor on): {parent_dir}/{filename}")
            print(f"    Alpha values: {alpha_dc}")
            print(f"    Mach values: {mach_dc}")
            print(f"    Reynolds values: {reynolds_values}")
            print(f"    Cases found: {len(cases)}")
            
            if cases and reynolds_values:
                create_datcom_ca_3d_csv(alpha_dc, mach_dc, cases, reynolds_values,
                                      os.path.join(output_dir, 'ca_3d_coeffs_motor_on.csv'))
            
            summary['datcom'] += 1
            print()
        except Exception as e:
            error_msg = f"Error processing DATCOM file {datcom_file}: {e}"
            print(f"  {error_msg}")
            summary['errors'].append(error_msg)
            print()
    
    # Process roll deflection DATCOM files -> roll_aero_coeffs.csv
    for datcom_file in sorted(datcom_roll):
        try:
            input_file = find_datcom_input_file(datcom_file)
            alpha_dc, mach_dc = parse_datcom_input_file(input_file) if input_file else ([], [])
            deflection_blocks = parse_datcom_deflection_blocks(datcom_file)
            
            filename = os.path.basename(datcom_file)
            parent_dir = os.path.basename(os.path.dirname(datcom_file))
            print(f"  Parsed DATCOM file (roll deflection): {parent_dir}/{filename}")
            print(f"    Alpha values: {alpha_dc}")
            print(f"    Mach values: {mach_dc}")
            print(f"    Deflection blocks found: {sorted(deflection_blocks.keys())}")
            
            if deflection_blocks:
                create_datcom_roll_csv_from_blocks(alpha_dc, mach_dc, deflection_blocks,
                                                  os.path.join(output_dir, 'roll_aero_coeffs.csv'))
            
            summary['datcom'] += 1
            print()
        except Exception as e:
            error_msg = f"Error processing DATCOM file {datcom_file}: {e}"
            print(f"  {error_msg}")
            summary['errors'].append(error_msg)
            print()
    
    # Process pitch deflection DATCOM files -> aero_coeffs.csv, damping_coeffs.csv, fin_deflection_coeffs.csv
    # Note: aero_coeffs.csv and damping_coeffs.csv use deflection=0 block as baseline
    for datcom_file in sorted(datcom_delta):
        try:
            input_file = find_datcom_input_file(datcom_file)
            alpha_dc, mach_dc = parse_datcom_input_file(input_file) if input_file else ([], [])
            deflection_blocks = parse_datcom_deflection_blocks(datcom_file)
            
            filename = os.path.basename(datcom_file)
            parent_dir = os.path.basename(os.path.dirname(datcom_file))
            print(f"  Parsed DATCOM file (pitch deflection): {parent_dir}/{filename}")
            print(f"    Alpha values: {alpha_dc}")
            print(f"    Mach values: {mach_dc}")
            print(f"    Deflection blocks found: {sorted(deflection_blocks.keys())}")
            
            if deflection_blocks:
                if 0.0 in deflection_blocks:
                    zero_coeffs = deflection_blocks[0.0]
                    print(f"    Using deflection=0 block for aero_coeffs.csv and damping_coeffs.csv")
                    print(f"    Coefficients in deflection=0 block: {list(zero_coeffs.keys())}")
                    
                    create_datcom_aero_csv(alpha_dc, mach_dc, zero_coeffs,
                                         os.path.join(output_dir, 'aero_coeffs.csv'))
                    
                    create_datcom_damping_csv(alpha_dc, mach_dc, zero_coeffs,
                                            os.path.join(output_dir, 'damping_coeffs.csv'))
                else:
                    print(f"    Warning: No deflection=0 block found, skipping aero_coeffs.csv and damping_coeffs.csv")
                
                create_datcom_fin_deflection_csv_from_blocks(alpha_dc, mach_dc, deflection_blocks,
                                                            os.path.join(output_dir, 'fin_deflection_coeffs.csv'))
            
            summary['datcom'] += 1
            print()
        except Exception as e:
            error_msg = f"Error processing DATCOM file {datcom_file}: {e}"
            print(f"  {error_msg}")
            summary['errors'].append(error_msg)
            print()
    
    # Process other DATCOM files (fallback)
    for datcom_file in sorted(datcom_other):
        try:
            alpha_dc, mach_dc, coefficients = parse_datcom_file(datcom_file)
            filename = os.path.basename(datcom_file)
            parent_dir = os.path.basename(os.path.dirname(datcom_file))
            print(f"  Parsed DATCOM file (other): {parent_dir}/{filename}")
            print(f"    Alpha values: {alpha_dc}")
            print(f"    Mach values: {mach_dc}")
            print(f"    Coefficients: {list(coefficients.keys())}")
            
            output_name = f'aero_coeffs_{parent_dir}.csv'
            create_datcom_aero_csv(alpha_dc, mach_dc, coefficients,
                                 os.path.join(output_dir, output_name))
            summary['datcom'] += 1
            print()
        except Exception as e:
            error_msg = f"Error processing DATCOM file {datcom_file}: {e}"
            print(f"  {error_msg}")
            summary['errors'].append(error_msg)
            print()
    
    print(f"  CSV files created in: {output_dir}")
    return summary


def main():
    """Main function that auto-detects and processes aerodynamic data files.
    
    The script can work in multiple modes:
    1. Single directory mode (default): Process only the directory where the script is located
    2. Multi-directory mode (--all): Process all sibling directories that contain aerodynamic data
    3. Single file mode (--file): Process a specific file and output to specified location
    
    Files are identified by their content structure, not their filenames.
    """
    parser = argparse.ArgumentParser(
        description='Convert aerodynamic .dat files and Excel files to CSV format.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python convert_aero_to_csv.py           # Process current directory only
  python convert_aero_to_csv.py --all     # Process all sibling directories
  python convert_aero_to_csv.py --dir /path/to/dir  # Process specific directory
  python convert_aero_to_csv.py --all --list  # List directories without processing
  python convert_aero_to_csv.py --excel   # Also convert Excel files to CSV
  python convert_aero_to_csv.py --excel-only  # Only convert Excel files (skip .dat files)
  python convert_aero_to_csv.py --file /path/to/file.dat --output /path/to/output  # Convert specific file
        """
    )
    parser.add_argument('--all', '-a', action='store_true',
                       help='Process all sibling directories that contain aerodynamic data')
    parser.add_argument('--dir', '-d', type=str, default=None,
                       help='Process a specific directory instead of the script location')
    parser.add_argument('--file', '-f', type=str, default=None,
                       help='Process a specific file (use with --output to specify output location)')
    parser.add_argument('--output', '-o', type=str, default=None,
                       help='Output directory for converted files (used with --file or --dir)')
    parser.add_argument('--list', '-l', action='store_true',
                       help='List directories that would be processed (with --all)')
    parser.add_argument('--excel', '-e', action='store_true',
                       help='Also convert Excel files (.xlsx, .xls) to CSV')
    parser.add_argument('--excel-only', action='store_true',
                       help='Only convert Excel files (skip .dat file processing)')
    
    args = parser.parse_args()
    
    # Get the directory where this script is located
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    print("=" * 60)
    print("Aerodynamic Data Converter")
    try:
        print("محول بيانات الديناميكا الهوائية")
    except UnicodeEncodeError:
        pass  # Skip Arabic text if encoding not supported
    print("=" * 60)
    
    # Single file mode: process a specific file
    # وضع الملف الواحد: معالجة ملف محدد
    if args.file:
        filepath = os.path.abspath(args.file)
        if not os.path.exists(filepath):
            print(f"\nError: File not found: {filepath}")
            return
        
        # Determine output directory
        # تحديد مجلد الإخراج
        if args.output:
            output_dir = os.path.abspath(args.output)
        else:
            output_dir = os.path.dirname(filepath)
        
        # Create output directory if it doesn't exist
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            print(f"\nCreated output directory: {output_dir}")
        
        print(f"\nSingle file mode")
        print(f"Input file: {filepath}")
        print(f"Output directory: {output_dir}")
        
        # Detect file type
        file_type = detect_file_type(filepath)
        if not file_type:
            print(f"\nError: Could not detect file type for: {filepath}")
            return
        
        print(f"Detected file type: {file_type.upper()}")
        
        # Process the file based on its type
        # معالجة الملف بناءً على نوعه
        try:
            if file_type == 'ca':
                alpha, mach, reynolds, ca_data = parse_ca_file(filepath)
                for i, (re_val, data) in enumerate(ca_data.items()):
                    output_file = os.path.join(output_dir, f'ca_3d_coeffs_{i+1}.csv')
                    create_ca_3d_csv(alpha, mach, re_val, data, output_file)
                    print(f"  Created: {output_file}")
                    
            elif file_type == 'pppcg':
                alpha, mach, coefficients = parse_pppcg_file(filepath)
                
                # Create aero_coeffs.csv
                aero_file = os.path.join(output_dir, 'aero_coeffs.csv')
                create_aero_coeffs_csv(alpha, mach, coefficients, aero_file)
                print(f"  Created: {aero_file}")
                
                # Create damping_coeffs.csv
                damping_file = os.path.join(output_dir, 'damping_coeffs.csv')
                create_damping_coeffs_csv(alpha, mach, coefficients, damping_file)
                print(f"  Created: {damping_file}")
                
                # Create fin_deflection_coeffs.csv
                fin_file = os.path.join(output_dir, 'fin_deflection_coeffs.csv')
                create_fin_deflection_coeffs_csv(alpha, mach, coefficients, fin_file)
                print(f"  Created: {fin_file}")
                
            elif file_type == 'rr':
                alpha, mach, roll, cll_data = parse_rr_file(filepath)
                output_file = os.path.join(output_dir, 'roll_aero_coeffs.csv')
                create_roll_aero_coeffs_csv(alpha, mach, roll, cll_data, output_file)
                print(f"  Created: {output_file}")
                
            elif file_type == 'datcom':
                # parse_datcom_file returns (alpha, mach, coefficients)
                alpha, mach, coefficients = parse_datcom_file(filepath)
                
                if not alpha or not mach:
                    print("  Warning: Could not extract alpha/mach values from DATCOM file")
                
                # Create aero_coeffs.csv
                aero_file = os.path.join(output_dir, 'aero_coeffs.csv')
                create_datcom_aero_csv(alpha, mach, coefficients, aero_file)
                print(f"  Created: {aero_file}")
                
                # Create damping_coeffs.csv
                damping_file = os.path.join(output_dir, 'damping_coeffs.csv')
                create_datcom_damping_csv(alpha, mach, coefficients, damping_file)
                print(f"  Created: {damping_file}")
            
            print(f"\nConversion complete!")
            print(f"Output files are in: {output_dir}")
            
        except Exception as e:
             print(f"\nError processing file: {e}")
        
        return
    
    # Determine which directory/directories to process
    if args.dir:
        directories = [os.path.abspath(args.dir)]
        # If output is specified with --dir, use it as the output directory
        # إذا تم تحديد الإخراج مع --dir، استخدمه كمجلد الإخراج
        if args.output:
            output_override = os.path.abspath(args.output)
            if not os.path.exists(output_override):
                os.makedirs(output_override)
                print(f"\nCreated output directory: {output_override}")
        else:
            output_override = None
    elif args.all:
        directories = find_sibling_directories(script_dir)
        output_override = None
    else:
        directories = [script_dir]
        output_override = None
    
    if args.all:
        print(f"\nMulti-directory mode: Processing all sibling directories")
        print(f"Found {len(directories)} potential directories:")
        for d in directories:
            has_data = has_aero_data(d)
            has_excel = len(find_excel_files(d)) > 0 if (args.excel or args.excel_only) else False
            if args.excel_only:
                status = "[HAS EXCEL]" if has_excel else "[NO EXCEL]"
            else:
                status = "[HAS DATA]" if has_data or has_excel else "[NO DATA]"
            print(f"  {status} {os.path.basename(d)}")
        
        if args.list:
            print("\n--list flag set, not processing files.")
            return
        
        # Filter to only directories with data
        if args.excel_only:
            directories = [d for d in directories if len(find_excel_files(d)) > 0]
        else:
            directories = [d for d in directories if has_aero_data(d) or 
                          ((args.excel or args.excel_only) and len(find_excel_files(d)) > 0)]
        print(f"\nWill process {len(directories)} directories with data.")
    else:
        print(f"\nSingle directory mode: {directories[0]}")
    
    # Process each directory
    total_summary = {'ca': 0, 'pppcg': 0, 'rr': 0, 'datcom': 0, 'excel': 0, 'errors': [], 'directories': 0}
    
    for directory in directories:
        dir_has_data = False
        
        if not args.excel_only:
            summary = process_directory(directory)
            total_summary['ca'] += summary['ca']
            total_summary['pppcg'] += summary['pppcg']
            total_summary['rr'] += summary['rr']
            total_summary['datcom'] += summary['datcom']
            total_summary['errors'].extend(summary['errors'])
            if summary['ca'] + summary['pppcg'] + summary['rr'] + summary['datcom'] > 0:
                dir_has_data = True
        
        if args.excel or args.excel_only:
            print(f"\n  Processing Excel files in: {os.path.basename(directory)}")
            excel_summary = process_excel_files(directory)
            total_summary['excel'] += excel_summary['converted']
            total_summary['errors'].extend(excel_summary['errors'])
            if excel_summary['converted'] > 0:
                dir_has_data = True
        
        if dir_has_data:
            total_summary['directories'] += 1
    
    # Print final summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"Directories processed: {total_summary['directories']}")
    if not args.excel_only:
        print(f"CA files converted: {total_summary['ca']}")
        print(f"PPPCG files converted: {total_summary['pppcg']}")
        print(f"RR files converted: {total_summary['rr']}")
        print(f"DATCOM files converted: {total_summary['datcom']}")
    if args.excel or args.excel_only:
        print(f"Excel files converted: {total_summary['excel']}")
    
    if total_summary['errors']:
        print(f"\nErrors encountered: {len(total_summary['errors'])}")
        for error in total_summary['errors']:
            print(f"  - {error}")


if __name__ == '__main__':
    main()
