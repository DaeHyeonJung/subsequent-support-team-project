import tkinter as tk
from tkinter import ttk
from typing import Callable


class FormationPanel:
    def __init__(self, parent: tk.Widget, on_shape_change: Callable[[str], None]):
        self.parent = parent
        self.on_shape_change = on_shape_change

        self.frame = ttk.Frame(parent)
        ttk.Label(self.frame, text="Formation Shape", font=("Arial", 12, "bold")).grid(
            row=0,
            column=0,
            sticky="w",
        )

        self.shape_var = tk.StringVar(value="")
        for i, shape in enumerate(["wedge", "line", "column", "staggered_column"]):
            ttk.Radiobutton(
                self.frame,
                text=shape.replace("_", " ").title(),
                variable=self.shape_var,
                value=shape,
                command=self._on_radio_change,
            ).grid(row=i + 1, column=0, sticky="w", pady=2)

    def _on_radio_change(self) -> None:
        self.on_shape_change(self.shape_var.get())

    def get_current_shape(self) -> str:
        return self.shape_var.get()

    def clear_selection(self) -> None:
        self.shape_var.set("")

    def set_selection(self, shape_type: str) -> None:
        self.shape_var.set(shape_type)
