from ortools.sat.python import cp_model
import time
import itertools

#GRAPH CONFIG
#Base Graph
GRID_WIDTH = 120 #The infinite path dimension length
GRID_HEIGHT = 5 #The cycle dimension (height of the prism, loops around)
MAX_COL_NUMBER = 119#This is the max coloring number which can be used in a graph.
TORODORIAL=True #If true, wraps horizontally to simulate an infinite path (Infinite Prism), making it a torus mathematically.
LOWEST_NUMBER=False #Instead of aiming for a single solution, this will aim for the lowest MAX coloring number solution.
#ENFORCE ____ RESTRICTIONS
PACKING=True #Enforces Packing Contraints(5 must be more than 5 away from another 5)
DOUBLES=True #Restricts doubling a--2a
SANDWICHES=True #Restricts Sandwiches a--b--a
STAIRS=True #Restrict staircases a--a+b--a+2b
IDENTICAL_NEIGHBORS=False #Restricts Identical Neighbors a--a(Only actually does something if packing is disabled)


#SEARCH CONFIG
NUM_SEARCH_WORKERS=16 #This is the number of threads your CPU will use for the python script(0 will use all)
MAX_MEMORY_IN_MB =16000 #Max memory, 0 for infinite
MAX_TIME_IN_MINUTES= 600 #Set to 0 for infinite time, but IDK why you would want that...
LOG_SEARCH=True #This honestly will just put gunk in your console
RANDOM_SEARCH_SEED=314 #No need to change this really, unless you're just desperate

#fixes improper max colorings for performance as anything greater is impossible
if TORODORIAL and MAX_COL_NUMBER >= GRID_WIDTH * GRID_HEIGHT:
    MAX_COL_NUMBER = GRID_WIDTH * GRID_HEIGHT - 1
    print(f"The max coloring number has been lowered to {MAX_COL_NUMBER} due to the bounds constraint")

def solve_infinite():
    model = cp_model.CpModel()
    
    # 1. CREATE VARIABLES
    grid = {}
    for r in range(GRID_HEIGHT):
        for c in range(GRID_WIDTH):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    print(f"Loading constraints for Prism {GRID_WIDTH} width x {GRID_HEIGHT} height...")
    start_time = time.time()

    # PRE-COMPUTATION OF EXCLUSION ZONES
    print("Pre-computing exclusion zones...", end=" ")
    offsets_by_z = {}
    
    for z in range(1, MAX_COL_NUMBER + 1):
        unique_offsets = set()
        
        # Optimize dy to only cover shortest paths on the cycle
        dy_min = -(GRID_HEIGHT - 1) // 2
        dy_max = GRID_HEIGHT // 2
        min_dy_possible = max(-z, dy_min)
        max_dy_possible = min(z, dy_max)
        
        for dy in range(min_dy_possible, max_dy_possible + 1):
            remaining = z - abs(dy)
            
            # Since TORODORIAL is true on solve_infinite, dx wraps too
            dx_min = -(GRID_WIDTH - 1) // 2
            dx_max = GRID_WIDTH // 2
            min_dx_possible = max(-remaining, dx_min)
            max_dx_possible = min(remaining, dx_max)
            
            for dx in range(min_dx_possible, max_dx_possible + 1):
                if dy == 0 and dx == 0: continue
                unique_offsets.add((dy, dx))
        offsets_by_z[z] = unique_offsets
    print("Done.")
 
    if PACKING:
        for z in range(1, MAX_COL_NUMBER + 1):
            if z % 5 == 0: # Progress bar
                print(f"  - Generating constraints for Color {z}/{MAX_COL_NUMBER}...")

            diamond_offsets = offsets_by_z[z]
            
            for r in range(GRID_HEIGHT):
                for c in range(GRID_WIDTH):

                    current_index= r * GRID_WIDTH + c
                    for dy, dx in diamond_offsets:
                        nr = (r + dy) % GRID_HEIGHT
                        nc = (c + dx) % GRID_WIDTH
                        neighbor_index= nr * GRID_WIDTH + nc

                        if neighbor_index > current_index:
                            model.AddForbiddenAssignments([grid[r,c],grid[nr,nc]],[(z,z)])

    # Total difference Labeling

    print("  - Generating Adjacency Rules...")
    # Offsets: Up, Down, Left, Right
    neighbor_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for r in range(GRID_HEIGHT):
        for c in range(GRID_WIDTH):
            current = grid[r, c]
            
            # Collect Neighbors 
            neighbors = []
            for dy, dx in neighbor_offsets:
                nr, nc = (r + dy) % GRID_HEIGHT, (c + dx) % GRID_WIDTH
                neighbors.append(grid[nr, nc])

            for neighbor in neighbors:
                # Rule 1: No Doubling
                if DOUBLES:
                    model.Add(neighbor != 2 * current)
                
                # Rule 2: Basic Inequality
                if IDENTICAL_NEIGHBORS and not PACKING:
                    model.Add(current != neighbor)

            # Rule 3 & 4: Staircase and Sandwich prevention
            for i in range(len(neighbors)):
                for j in range(i + 1, len(neighbors)):
                    n1 = neighbors[i]
                    n2 = neighbors[j]
                    if STAIRS:
                        model.Add(2 * current != n1 + n2)
                    if SANDWICHES:
                        model.Add(n1 != n2)

    load_time = time.time() - start_time
    print(f"Constraints loaded in {load_time:.2f} seconds.")

    #PERFORMANCE STUFF
    if LOWEST_NUMBER:
        print("not yet set up to find lowest number")
    else:
        #This will force the MAX Col Number to be placed. and then will automatically stop every other square from being it.
        model.Add(grid[0,0] == MAX_COL_NUMBER)
        
    #Capacity bounds bypassed for general prism
    for z in range(1, MAX_COL_NUMBER+1):
        maxCount = get_max_capacity(z, GRID_WIDTH, GRID_HEIGHT)
        if maxCount is None:
            continue
        print(f"Bounding Color {z} to at most {maxCount} occurances")
        zPresenceBooleans = []

        for r in range(GRID_HEIGHT):
            for c in range(GRID_WIDTH):
                b_is_z = model.NewBoolVar(f'is_{z}_{r}_{c}')

                model.Add(grid[r,c]==z).OnlyEnforceIf(b_is_z)
                model.Add(grid[r,c]!=z).OnlyEnforceIf(b_is_z.Not())
                
                zPresenceBooleans.append(b_is_z)

        if maxCount == 1:
            model.AddAtMostOne(zPresenceBooleans)
        else:
            model.Add(sum(zPresenceBooleans) <= maxCount)
        



    # ---------------------------------------------------------
    # SOLVE
    # ---------------------------------------------------------
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = NUM_SEARCH_WORKERS
    solver.parameters.max_memory_in_mb = MAX_MEMORY_IN_MB
    solver.parameters.max_time_in_seconds = MAX_TIME_IN_MINUTES*60.0
    solver.parameters.random_seed = RANDOM_SEARCH_SEED
    solver.parameters.log_search_progress = LOG_SEARCH

    print("Solving...")
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f"SUCCESS: Solution Found for Prism {GRID_WIDTH} width x {GRID_HEIGHT} height!")
        for r in range(GRID_HEIGHT):
            row = [solver.Value(grid[r, c]) for c in range(GRID_WIDTH)]
            print(row)
    else:
        print("No solution found.")

def solve_finite():
    print(f"\n[{time.strftime('%H:%M:%S')}] Starting setup for Max Colors: {MAX_COL_NUMBER}")
    model = cp_model.CpModel()
    
    # 1. CREATE VARIABLES
    print(f"  > Creating variables for Prism {GRID_WIDTH} width x {GRID_HEIGHT} height...")
    grid = {}
    for r in range(GRID_HEIGHT):
        for c in range(GRID_WIDTH):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    # ---------------------------------------------------------
    # A. PACKING CONSTRAINTS (Finite Exclusion Zone)
    # ---------------------------------------------------------
    if PACKING:
        print("  > Building Packing Constraints...")
        
        # Precompute the shortest path offsets bounded by cycle size
        dy_min = -(GRID_HEIGHT - 1) // 2
        dy_max = GRID_HEIGHT // 2
        
        for z in range(1, MAX_COL_NUMBER + 1):
            if z % 10 == 0 or z == MAX_COL_NUMBER:
                print(f"    ... processing color {z}/{MAX_COL_NUMBER}")
                
            min_dy_possible = max(-z, dy_min)
            max_dy_possible = min(z, dy_max)
            
            for r in range(GRID_HEIGHT):
                for c in range(GRID_WIDTH):
                    b = model.NewBoolVar(f'is_{z}_{r}_{c}')
                    model.Add(grid[r, c] == z).OnlyEnforceIf(b)
                    model.Add(grid[r, c] != z).OnlyEnforceIf(b.Not())

                    for dy in range(min_dy_possible, max_dy_possible + 1):
                        remaining = z - abs(dy)
                        for dx in range(-remaining, remaining + 1):
                            if dy == 0 and dx == 0: continue
                            
                            nr = (r + dy) % GRID_HEIGHT
                            nc = c + dx
                            
                            # BOUNDARY CHECK: Only apply if neighbor is inside grid horizontally
                            if 0 <= nc < GRID_WIDTH:
                                model.Add(grid[nr, nc] != z).OnlyEnforceIf(b)

    # ---------------------------------------------------------
    # C. VERTEX ADJACENCY RULES
    # ---------------------------------------------------------
    print("  > Building Vertex Adjacency & Arithmetic Rules...")
    neighbor_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for r in range(GRID_HEIGHT):
        for c in range(GRID_WIDTH):
            current = grid[r, c]
            
            # 1. Collect Valid Neighbors
            valid_neighbors = []
            for dy, dx in neighbor_offsets:
                nr = (r + dy) % GRID_HEIGHT
                nc = c + dx
                if 0 <= nc < GRID_WIDTH:
                    valid_neighbors.append(grid[nr, nc])

            # 2. Apply Rules to Valid Neighbors
            for neighbor in valid_neighbors:
                # Rule 1: No Doubling
                if DOUBLES:
                    model.Add(current != 2 * neighbor)
                
                # Rule 2: Basic Inequality
                if IDENTICAL_NEIGHBORS and not PACKING:
                    model.Add(current != neighbor)

            # Rule 3: Arithmetic Progression Prevention
            for i in range(len(valid_neighbors)):
                for j in range(i + 1, len(valid_neighbors)):
                    n1 = valid_neighbors[i]
                    n2 = valid_neighbors[j]
                    if STAIRS:
                        model.Add(2 * current != n1 + n2)
                    if SANDWICHES:
                        model.Add(n1 != n2)



    # ---------------------------------------------------------
    # SOLVE
    # ---------------------------------------------------------
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = NUM_SEARCH_WORKERS
    solver.parameters.random_seed = RANDOM_SEARCH_SEED
    solver.parameters.max_memory_in_mb = MAX_MEMORY_IN_MB
    solver.parameters.max_time_in_seconds = MAX_TIME_IN_MINUTES*60.0
    solver.parameters.log_search_progress = LOG_SEARCH

    print(f"  > Model built. Starting CP-SAT Solver (Time limit: {MAX_TIME_IN_MINUTES})...")
    start_time = time.time()
    status = solver.Solve(model)
    elapsed = time.time() - start_time

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f"  >>> SUCCESS: Solution Found in {elapsed:.2f} seconds!")
        
        final_grid = []
        for r in range(GRID_HEIGHT):
            row = [solver.Value(grid[r, c]) for c in range(GRID_WIDTH)]
            final_grid.append(row)
            print(f"   {row}")
        return True
    else:
        print(f"  >>> FAILURE: No solution found within constraints (Time: {elapsed:.2f}s).")
        return False

def get_max_capacity(z, grid_width, grid_height):
    # Omitted capacity constraints by default since they were torus-specific.
    # We can add prism-specific heuristics later.
    return None

if __name__ == "__main__":
    if TORODORIAL:
        solve_infinite()
    else:
        solve_finite()
