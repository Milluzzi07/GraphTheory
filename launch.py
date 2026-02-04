import os
import sys
import subprocess
import json
from pathlib import Path

# --- CONFIGURATION ---
VENV_NAME = ".venv"
REQUIRED_PACKAGES = ["numpy", "ortools"]
# ---------------------

def get_venv_paths(root_dir):
    """Returns paths for venv root, scripts, and python executable."""
    if os.name == 'nt': # Windows
        scripts_dir = root_dir / VENV_NAME / "Scripts"
        python_exe = scripts_dir / "python.exe"
    else: # Mac/Linux
        scripts_dir = root_dir / VENV_NAME / "bin"
        python_exe = scripts_dir / "python"
        
    return {
        "root": root_dir / VENV_NAME,
        "scripts": scripts_dir,
        "python": python_exe
    }

def create_venv_if_missing(root_dir):
    venv_path = root_dir / VENV_NAME
    if not venv_path.exists():
        print(f"--- Creating virtual environment '{VENV_NAME}'... ---")
        subprocess.check_call([sys.executable, "-m", "venv", str(venv_path)])
    else:
        print("--- Virtual environment found. ---")

def install_dependencies(venv_python):
    print("--- Verifying dependencies... ---")
    cmd = [str(venv_python), "-m", "pip", "install"] + REQUIRED_PACKAGES
    subprocess.check_call(cmd, stdout=subprocess.DEVNULL)
    print("--- Dependencies ready. ---")

def configure_vscode(root_dir, venv_python):
    """Writes the .vscode/settings.json file to force interpreter selection."""
    vscode_dir = root_dir / ".vscode"
    settings_path = vscode_dir / "settings.json"
    
    # Create .vscode folder if it doesn't exist
    vscode_dir.mkdir(exist_ok=True)
    
    # Prepare the setting (Using ${workspaceFolder} is best for portability)
    # But since we are generating it, we can use the relative path
    relative_python_path = os.path.join(VENV_NAME, "Scripts", "python.exe") if os.name == 'nt' else os.path.join(VENV_NAME, "bin", "python")
    
    new_settings = {
        "python.defaultInterpreterPath": f"${{workspaceFolder}}/{relative_python_path}",
        # Optional: Setup analysis tools to use this venv too
        "python.analysis.extraPaths": [f"${{workspaceFolder}}/{VENV_NAME}/Lib/site-packages"]
    }

    current_settings = {}
    
    # If file exists, read it to avoid deleting other settings
    if settings_path.exists():
        try:
            with open(settings_path, "r") as f:
                current_settings = json.load(f)
        except json.JSONDecodeError:
            print("Warning: existing settings.json was corrupt. Overwriting.")
    
    # Merge new settings (overwriting conflicts)
    current_settings.update(new_settings)
    
    with open(settings_path, "w") as f:
        json.dump(current_settings, f, indent=4)
        
    print(f"--- VS Code configured to use {VENV_NAME}. ---")

def spawn_activated_shell(venv_paths):
    print("\n>>> Environment Ready. Spawning Shell.")
    print(">>> VS Code has been updated. You may need to restart VS Code for it to apply.")
    
    env = os.environ.copy()
    env["PATH"] = str(venv_paths["scripts"]) + os.pathsep + env["PATH"]
    env["VIRTUAL_ENV"] = str(venv_paths["root"])
    if "PYTHONHOME" in env: del env["PYTHONHOME"]

    subprocess.call(["powershell"], env=env)

def main():
    project_dir = Path(__file__).parent.resolve()
    venv_paths = get_venv_paths(project_dir)

    # 1. Setup Venv
    create_venv_if_missing(project_dir)
    
    # 2. Install Packages
    install_dependencies(venv_paths["python"])
    
    # 3. Force VS Code to use this Venv
    configure_vscode(project_dir, venv_paths["python"])

    # 4. Launch Shell
    spawn_activated_shell(venv_paths)

if __name__ == "__main__":
    main()