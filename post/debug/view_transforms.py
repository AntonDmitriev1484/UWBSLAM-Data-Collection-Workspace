import os
import pickle
import matplotlib.pyplot as plt

# List all .pkl files in the current directory
pickle_files = [f for f in os.listdir('.') if f.endswith('.pkl')]

if not pickle_files:
    print("No .pkl files found.")
    exit()

for pkl_file in pickle_files:
    print(f"Opening: {pkl_file}")
    try:
        with open(pkl_file, 'rb') as f:
            figs = pickle.load(f)

        if isinstance(figs, dict):
            for name, fig in figs.items():
                print(f"Showing figure: {name}")
                fig.show()
                plt.pause(0.001)  # Allow GUI event loop to run
                input("Press Enter to continue to next figure...")
        elif isinstance(figs, plt.Figure):
            figs.show()
            input("Press Enter to continue to next figure...")
        else:
            print(f"{pkl_file} does not contain a valid Matplotlib figure or dict of figures.")
    except Exception as e:
        print(f"Error loading {pkl_file}: {e}")
