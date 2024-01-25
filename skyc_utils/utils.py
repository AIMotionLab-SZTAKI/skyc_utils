import tkinter as tk
from tkinter import filedialog
from functools import partial
from typing import List, Optional, Union
import os
import shutil


def open_file_dialog(file_path_var: List[Optional[str]], root: tk.Tk, filetype: str) -> None:
    """
    Helper function for select_file, which handles the selection window.
    """
    file_path = filedialog.askopenfilename(initialdir=os.path.dirname(__file__),
                                           title=f"Select a {filetype} file!",
                                           filetypes=[(f"{filetype} files", f"*.{filetype}"), ("all files", "*.*")])
    if file_path:
        print(f"Selected file: {file_path}")
        file_path_var[0] = file_path
        root.destroy()


def select_file(filetype: str) -> Union[None, str]:
    """ Function that prompts the user to select a skyc file, and returns the file's name if successful.
    Else returns None"""
    selecter = tk.Tk()
    selecter.title(f"Select {filetype} file!")
    selecter.geometry("300x100")
    # using a list to mimic a mutable object (I hate python so much why are you forcing me to
    # do this, just let me pass this by reference, horrible toy language for children and phds...)
    selected_file = [None]
    button_func = partial(open_file_dialog, selected_file, selecter, filetype)
    button = tk.Button(selecter, text=f"Select {filetype} file!", command=button_func, width=20, height=4)
    button.pack(pady=20)
    selecter.mainloop()
    return selected_file[0]


def cleanup(files: List[str], folders: List[str]):
    """
    function meant for deleting unnecessary files
    """
    for file in files:
        if os.path.exists(file):
            os.remove(file)
            print(f"Deleted {file}")
    for folder in folders:
        if os.path.exists(folder):
            shutil.rmtree(folder)
            print(f"Deleted {folder} folder")