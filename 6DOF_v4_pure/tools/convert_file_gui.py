#!/usr/bin/env python3
"""
GUI File Converter - Convert aerodynamic data files to CSV format.
محول الملفات بواجهة رسومية - تحويل ملفات البيانات الديناميكية الهوائية إلى صيغة CSV.

This tool provides a graphical interface to select a file and convert it to CSV format.
The output files are saved in the same directory as the input file.

توفر هذه الأداة واجهة رسومية لاختيار ملف وتحويله إلى صيغة CSV.
يتم حفظ ملفات الإخراج في نفس المجلد الذي يوجد فيه الملف الأصلي.

Usage:
    python convert_file_gui.py
    
    A file dialog will open to select the file to convert.
    ستفتح نافذة حوار لاختيار الملف المراد تحويله.
"""

import os
import sys
import tkinter as tk
from tkinter import filedialog, messagebox

# Add parent directory to path to import convert_aero_to_csv functions
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from convert_aero_to_csv import (
    detect_file_type,
    parse_ca_file,
    parse_pppcg_file,
    parse_rr_file,
    parse_datcom_file,
    create_ca_3d_csv,
    create_aero_coeffs_csv,
    create_damping_coeffs_csv,
    create_roll_aero_coeffs_csv,
    create_fin_deflection_coeffs_csv,
    create_datcom_aero_csv,
    create_datcom_damping_csv,
)


def convert_file(filepath):
    """
    Convert a single aerodynamic data file to CSV format.
    تحويل ملف بيانات ديناميكية هوائية واحد إلى صيغة CSV.
    
    Args:
        filepath: Path to the input file
        
    Returns:
        tuple: (success: bool, message: str, output_files: list)
    """
    if not os.path.exists(filepath):
        return False, f"File not found: {filepath}\nالملف غير موجود: {filepath}", []
    
    # Output directory is the same as the input file directory
    # مجلد الإخراج هو نفس مجلد الملف المدخل
    output_dir = os.path.dirname(filepath)
    
    # Detect file type
    # كشف نوع الملف
    file_type = detect_file_type(filepath)
    if not file_type:
        return False, f"Could not detect file type.\nلم يتم التعرف على نوع الملف.", []
    
    output_files = []
    
    try:
        if file_type == 'ca':
            alpha, mach, reynolds, ca_data = parse_ca_file(filepath)
            for i, (re_val, data) in enumerate(ca_data.items()):
                output_file = os.path.join(output_dir, f'ca_3d_coeffs_{i+1}.csv')
                create_ca_3d_csv(alpha, mach, re_val, data, output_file)
                output_files.append(output_file)
                
        elif file_type == 'pppcg':
            alpha, mach, coefficients = parse_pppcg_file(filepath)
            
            # Create aero_coeffs.csv
            aero_file = os.path.join(output_dir, 'aero_coeffs.csv')
            create_aero_coeffs_csv(alpha, mach, coefficients, aero_file)
            output_files.append(aero_file)
            
            # Create damping_coeffs.csv
            damping_file = os.path.join(output_dir, 'damping_coeffs.csv')
            create_damping_coeffs_csv(alpha, mach, coefficients, damping_file)
            output_files.append(damping_file)
            
            # Create fin_deflection_coeffs.csv
            fin_file = os.path.join(output_dir, 'fin_deflection_coeffs.csv')
            create_fin_deflection_coeffs_csv(alpha, mach, coefficients, fin_file)
            output_files.append(fin_file)
            
        elif file_type == 'rr':
            alpha, mach, roll, cll_data = parse_rr_file(filepath)
            output_file = os.path.join(output_dir, 'roll_aero_coeffs.csv')
            create_roll_aero_coeffs_csv(alpha, mach, roll, cll_data, output_file)
            output_files.append(output_file)
            
        elif file_type == 'datcom':
            alpha, mach, coefficients = parse_datcom_file(filepath)
            
            # Create aero_coeffs.csv
            aero_file = os.path.join(output_dir, 'aero_coeffs.csv')
            create_datcom_aero_csv(alpha, mach, coefficients, aero_file)
            output_files.append(aero_file)
            
            # Create damping_coeffs.csv
            damping_file = os.path.join(output_dir, 'damping_coeffs.csv')
            create_datcom_damping_csv(alpha, mach, coefficients, damping_file)
            output_files.append(damping_file)
        
        # Build success message
        # بناء رسالة النجاح
        files_list = '\n'.join([f'  - {os.path.basename(f)}' for f in output_files])
        message = (
            f"Conversion successful!\nاكتملت عملية التحويل بنجاح!\n\n"
            f"File type detected: {file_type.upper()}\n"
            f"نوع الملف المكتشف: {file_type.upper()}\n\n"
            f"Output files created:\nالملفات المنشأة:\n{files_list}\n\n"
            f"Location: {output_dir}\nالموقع: {output_dir}"
        )
        return True, message, output_files
        
    except Exception as e:
        return False, f"Error during conversion:\nخطأ أثناء التحويل:\n{str(e)}", []


def main():
    """
    Main function - opens file dialog and converts selected file.
    الدالة الرئيسية - تفتح نافذة اختيار الملف وتحول الملف المختار.
    """
    # Create root window (hidden)
    # إنشاء النافذة الرئيسية (مخفية)
    root = tk.Tk()
    root.withdraw()
    
    # Set window title for dialogs
    # تعيين عنوان النافذة للحوارات
    root.title("File Converter - محول الملفات")
    
    # Open file dialog
    # فتح نافذة اختيار الملف
    filepath = filedialog.askopenfilename(
        title="Select file to convert - اختر الملف للتحويل",
        filetypes=[
            ("All supported files", "*.dat *.DAT *.txt *.TXT"),
            ("DAT files", "*.dat *.DAT"),
            ("Text files", "*.txt *.TXT"),
            ("All files", "*.*")
        ]
    )
    
    # Check if user cancelled
    # التحقق من إلغاء المستخدم
    if not filepath:
        messagebox.showinfo(
            "Cancelled - تم الإلغاء",
            "No file selected.\nلم يتم اختيار أي ملف."
        )
        root.destroy()
        return
    
    # Convert the file
    # تحويل الملف
    success, message, output_files = convert_file(filepath)
    
    # Show result
    # عرض النتيجة
    if success:
        messagebox.showinfo("Success - نجاح", message)
    else:
        messagebox.showerror("Error - خطأ", message)
    
    root.destroy()


if __name__ == '__main__':
    main()
