from ortools.sat.python import cp_model
import time

# --- CONFIGURATION ---
GRID_SIZE = 30  # Set to 50 for your full run
MAX_COL_NUMBER = GRID_SIZE - 1 
N = GRID_SIZE

# --- CALLBACK CLASS ---
class GridSolutionPrinter(cp_model.CpSolverSolutionCallback):
    def __init__(self, grid, n, filename="solutions.txt"):
        cp_model.CpSolverSolutionCallback.__init__(self)
        self._grid = grid
        self._n = n
        self._filename = filename
        self._solution_count = 0
        self._start_time = time.time()

    def OnSolutionCallback(self):
        self._solution_count += 1
        elapsed = time.time() - self._start_time
        
        
        print(f"\n🎉 SOLUTION {self._solution_count} FOUND IN {elapsed:.2f} SECONDS! 🎉")
        for r in range(self._n):
            row = [self.Value(self._grid[r, c]) for c in range(self._n)]
            print(row)
        print("-" * 40)
        
        # 2. Save to a text file
        with open(self._filename, 'a') as f:
            f.write(f"Solution {self._solution_count} found in {elapsed:.2f} seconds:\n")
            for r in range(self._n):
                row = [self.Value(self._grid[r, c]) for c in range(self._n)]
                f.write(str(row) + '\n')
            f.write("-" * 40 + '\n')
            
        # 3. Halt the solver immediately
        self.StopSearch()

def solve_infinite_optimized():
    model = cp_model.CpModel()
    
    # 1. CREATE VARIABLES
    grid = {}
    for r in range(N):
        for c in range(N):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    print(f"Loading Toroidal constraints for {N}x{N}...")
    start_time = time.time()

    # --- PRE-COMPUTATION (The Speed Fix) ---
    print("Pre-computing diamond shapes...", end=" ")
    offsets_by_z = {}
    
    for z in range(1, MAX_COL_NUMBER + 1):
        unique_offsets = set()
        for dy in range(-z, z + 1):
            remaining = z - abs(dy)
            for dx in range(-remaining, remaining + 1):
                if dy == 0 and dx == 0: continue
                unique_offsets.add((dy, dx))
        offsets_by_z[z] = unique_offsets
    print("Done.")

    # ---------------------------------------------------------
    # A. PACKING CONSTRAINTS (Optimized)
    # ---------------------------------------------------------
    for z in range(1, MAX_COL_NUMBER + 1):
        if z % 5 == 0: # Progress bar
            print(f"  - Generating constraints for Color {z}/{MAX_COL_NUMBER}...")

        diamond_offsets = offsets_by_z[z]
        
        for r in range(N):
            for c in range(N):
                # 1. Create the Switch
                b_is_z = model.NewBoolVar(f'is_{z}_{r}_{c}')
                model.Add(grid[r, c] == z).OnlyEnforceIf(b_is_z)
                model.Add(grid[r, c] != z).OnlyEnforceIf(b_is_z.Not())

                # 2. Apply Exclusion Zone
                current_index = r * N + c 
                
                for dy, dx in diamond_offsets:
                    nr = (r + dy) % N
                    nc = (c + dx) % N
                    neighbor_index = nr * N + nc

                    # SYMMETRY OPTIMIZATION
                    if neighbor_index > current_index:
                        model.Add(grid[nr, nc] != z).OnlyEnforceIf(b_is_z)

    # ---------------------------------------------------------
    # B. VERTEX ADJACENCY RULES (Toroidal)
    # ---------------------------------------------------------
    print("  - Generating Adjacency Rules...")
    neighbor_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for r in range(N):
        for c in range(N):
            current = grid[r, c]
            
            neighbors = []
            for dy, dx in neighbor_offsets:
                nr, nc = (r + dy) % N, (c + dx) % N
                neighbors.append(grid[nr, nc])

            for neighbor in neighbors:
                # Rule 1: No Doubling
                # model.Add(neighbor != 2 * current)
                # Rule 2: Basic Inequality
                model.Add(current != neighbor)

            # Rule 3: Arithmetic Progression Prevention
            for i in range(len(neighbors)):
                for j in range(i + 1, len(neighbors)):
                    n1 = neighbors[i]
                    n2 = neighbors[j]
                    model.Add(2 * current != n1 + n2)

    load_time = time.time() - start_time
    print(f"Constraints loaded in {load_time:.2f} seconds.")

    #THIS NEXT PART IS USED TO TRY TO GET THE LOWEST POSSIBLE COLORING.
    # # Create a new variable to represent the maximum value on the board
    # max_val = model.NewIntVar(1, MAX_COL_NUMBER, 'max_val')
    
    # # Link this variable to the grid variables using AddMaxEquality
    # # This forces max_val to equal the highest number actually placed on the grid
    # model.AddMaxEquality(max_val, [grid[r, c] for r in range(N) for c in range(N)])
    
    # # Tell the solver to minimize this value
    # model.Minimize(max_val)
    # ---------------------------------------------------------
    # SOLVE
    # ---------------------------------------------------------
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = 16
    solver.parameters.max_memory_in_mb = 16000
    solver.parameters.max_time_in_seconds = 600.0
    
    # We leave this True so you can see the solver working in the background
    solver.parameters.log_search_progress = True 

    print("Solving...")
    
    # Initialize the callback
    solution_printer = GridSolutionPrinter(grid, N)
    
    # Run the solver WITH the callback attached
    status = solver.Solve(model, solution_printer)

    # Final wrap-up message
    if status == cp_model.OPTIMAL:
        print("\nFinished: Optimal solution proved (or search space exhausted).")
    elif status == cp_model.FEASIBLE:
        print("\nFinished: Time limit reached, but at least one feasible solution was found.")
    else:
        print("\nFinished: No solution found.")

if __name__ == "__main__":
    solve_infinite_optimized()