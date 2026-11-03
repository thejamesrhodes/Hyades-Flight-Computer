#!/usr/bin/env python3
"""make_shortcut.py -- put a 'Hyades FC-V4 Console' shortcut on the Windows desktop.

    python Tools/make_shortcut.py

Runs fc_console.py with pythonw (no console window), with its own icon.
Uses the Windows Script Host COM object through PowerShell; nothing installed.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))


def main() -> None:
    if not sys.platform.startswith("win"):
        sys.exit("Windows only")
    pyw = os.path.join(os.path.dirname(sys.executable), "pythonw.exe")
    if not os.path.isfile(pyw):
        pyw = sys.executable
    q = lambda s: s.replace("'", "''")                       # PowerShell single-quote escape
    ps = (
        "$d=[Environment]::GetFolderPath('Desktop');"
        "$s=(New-Object -ComObject WScript.Shell).CreateShortcut((Join-Path $d 'Hyades FC-V4 Console.lnk'));"
        f"$s.TargetPath='{q(pyw)}';"
        f"$s.Arguments='\"{q(os.path.join(HERE, 'fc_console.py'))}\"';"
        f"$s.WorkingDirectory='{q(HERE)}';"
        f"$s.IconLocation='{q(os.path.join(HERE, 'fc_console.ico'))}';"
        "$s.Description='Hyades FC-V4 bench/field console';"
        "$s.Save(); Write-Output (Join-Path $d 'Hyades FC-V4 Console.lnk')"
    )
    r = subprocess.run(["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
                       capture_output=True, text=True)
    print(r.stdout.strip() or r.stderr.strip())


if __name__ == "__main__":
    main()
