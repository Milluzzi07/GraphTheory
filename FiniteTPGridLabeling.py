import numpy as np

GRID_SIZE = 100
MAX_COL_NUMBER = 100 
TOTAL_CELLS = GRID_SIZE * GRID_SIZE

def solve_finite_grid():
    for colNum in range(1, MAX_COL_NUMBER + 1):
        print(f"Testing for max label: {colNum}...")
        grid = np.zeros(TOTAL_CELLS, dtype=int)
        
        # Start search from the first cell (No symmetry restrictions)
        if backtrack(grid, 0, colNum):
            print(f"SUCCESS: Found valid labeling with max number: {colNum}")
            print(grid.reshape((GRID_SIZE, GRID_SIZE)))
            return

    print("Unable to find a valid labeling for this finite grid.")

def backtrack(grid, idx, max_col):
    if idx == TOTAL_CELLS:
        return True 

    r, c = divmod(idx, GRID_SIZE)

    for val in range(1, max_col + 1):
        grid[idx] = val
        
        # Validates both vertex and edge-difference rules
        if is_locally_valid_finite(grid, r, c, val):
            if backtrack(grid, idx + 1, max_col):
                return True
                
        grid[idx] = 0

    return False

def is_locally_valid_finite(grid_1d, r, c, val):
    # --- 1. Packing Constraint (Manhattan Distance) ---
    for x in range(val + 1):
        y = val - x
        if x == 0 and y == 0: continue
        check_r, check_c = r - y, c - x
        if check_r >= 0 and check_c >= 0:
            if grid_1d[check_r * GRID_SIZE + check_c] == val:
                return False

    diff_left = -1
    diff_top = -1

    # --- 2. Left Edge Constraints ---
    if c > 0:
        left_val = grid_1d[r * GRID_SIZE + (c - 1)]
        diff_left = abs(val - left_val)
        
        # Vertex rules: Diff cannot be 0, or equal to the vertices it connects
        if diff_left == 0 or diff_left == val or diff_left == left_val: return False
            
        # Incident Edge rules: Diff cannot equal other edges touching (r, c-1)
        if r > 0: # Up edge from left neighbor
            up_left_val = grid_1d[(r - 1) * GRID_SIZE + (c - 1)]
            if diff_left == abs(left_val - up_left_val): return False
        if c > 1: # Left edge from left neighbor
            left2_val = grid_1d[r * GRID_SIZE + (c - 2)]
            if diff_left == abs(left_val - left2_val): return False

    # --- 3. Top Edge Constraints ---
    if r > 0:
        top_val = grid_1d[(r - 1) * GRID_SIZE + c]
        diff_top = abs(val - top_val)
        
        # Vertex rules
        if diff_top == 0 or diff_top == val or diff_top == top_val: return False

        # Incident Edge rules: Diff cannot equal other edges touching (r-1, c)
        if r > 1: # Up edge from top neighbor
            up2_val = grid_1d[(r - 2) * GRID_SIZE + c]
            if diff_top == abs(top_val - up2_val): return False
        if c > 0: # Left edge from top neighbor
            up_left_val = grid_1d[(r - 1) * GRID_SIZE + (c - 1)]
            if diff_top == abs(top_val - up_left_val): return False
        if c < GRID_SIZE - 1: # Right edge from top neighbor
            up_right_val = grid_1d[(r - 1) * GRID_SIZE + (c + 1)]
            if diff_top == abs(top_val - up_right_val): return False

    # --- 4. Current Vertex Rule ---
    # The two new edges being created right now cannot equal each other
    if c > 0 and r > 0:
        if diff_left == diff_top: return False

    return True

if __name__ == "__main__":
    solve_finite_grid()