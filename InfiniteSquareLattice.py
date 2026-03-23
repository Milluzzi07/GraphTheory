from ortools.sat.python import cp_model
import time
import itertools

#GRAPH CONFIG
#Base Graph
GRID_SIZE = 30 #The width and height of your graph
MAX_COL_NUMBER = 20 #This is the max coloring number which can be used in a graph. Note: On torodorial this is a max of GRID_SIZE-1
TORODORIAL=True #Want to find something infinite?
LOWEST_NUMBER=False #Instead of aiming for a single solution, this will aim for the lowest MAX coloring number solution. This will most likely use up all the time you give it.
#ENFORCE ____ RESTRICTIONS
PACKING=True #Enforces Packing Contraints(5 must be more than 5 away from another 5)
DOUBLES=False #Restricts doubling a--2a
SANDWICHES=False #Restricts Sandwiches a--b--a
STAIRS=False #Restrict staircases a--a+b--a+2b
IDENTICAL_NEIGHBORS=False #Restricts Identical Neighbors a--a(Only actually does something if packing is disabled)


#SEARCH CONFIG
NUM_SEARCH_WORKERS=0 #This is the number of threads your CPU will use for the python script(0 will use all)
MAX_MEMORY_IN_MB =16000 #Max memory, 0 for infinite
MAX_TIME_IN_MINUTES= 600 #Set to 0 for infinite time, but IDK why you would want that...
LOG_SEARCH=True #This honestly will just put gunk in your console
RANDOM_SEARCH_SEED=314 #No need to change this really, unless you're just desperate

#fixes improper max colorings for performance as anything greater is impossible
if TORODORIAL and MAX_COL_NUMBER>=GRID_SIZE:
    MAX_COL_NUMBER=GRID_SIZE-1
    print(f"The max coloring number has been lowered to {MAX_COL_NUMBER} due to the torodorial constraint")
def solve_infinite():
    model = cp_model.CpModel()
    
    # 1. CREATE VARIABLES
    grid = {}
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    print(f"Loading constraints for {GRID_SIZE}x{GRID_SIZE}...")
    start_time = time.time()

    # PRE-COMPUTATION OF EXCLUSION ZONES
    print("Pre-computing exclusion zones...", end=" ")
    offsets_by_z = {}
    
    for z in range(1, MAX_COL_NUMBER + 1):
        unique_offsets = set()
        for dy in range(-z, z + 1):
            remaining = z - abs(dy)
            for dx in range(-remaining, remaining + 1):
                if dy == 0 and dx == 0: continue
                # We store just the offsets. Wrapping happens later
                unique_offsets.add((dy, dx))
        offsets_by_z[z] = unique_offsets
    print("Done.")
 
    if PACKING:
        for z in range(1, MAX_COL_NUMBER + 1):
            if z % 5 == 0: # Progress bar
                print(f"  - Generating constraints for Color {z}/{MAX_COL_NUMBER}...")

            diamond_offsets = offsets_by_z[z]
            
            for r in range(GRID_SIZE):
                for c in range(GRID_SIZE):

                    current_index= r*GRID_SIZE + c
                    for dy, dx in diamond_offsets:
                        nr = (r + dy) % GRID_SIZE
                        nc = (c + dx) % GRID_SIZE
                        neighbor_index= nr*GRID_SIZE+nc

                        if neighbor_index > current_index:
                            model.AddForbiddenAssignments([grid[r,c],grid[nr,nc]],[(z,z)])

    # Total difference Labeling

    print("  - Generating Adjacency Rules...")
    # Offsets: Up, Down, Left, Right
    neighbor_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            current = grid[r, c]
            
            # Collect Neighbors 
            neighbors = []
            for dy, dx in neighbor_offsets:
                nr, nc = (r + dy) % GRID_SIZE, (c + dx) % GRID_SIZE
                neighbors.append(grid[nr, nc])

            for neighbor in neighbors:
                # Rule 1: No Doubling
                if DOUBLES:
                    model.Add(neighbor != 2 * current)
                # Note: We don't check 'current != 2 * neighbor' here because 
                # that constraint will be added when we iterate to 'neighbor'.
                
                # Rule 2: Basic Inequality
                if IDENTICAL_NEIGHBORS and not PACKING:
                    model.Add(current != neighbor)

            # Rule 3 & 4: Staircase and Sandwich prevention
            # "2 * Current != Neighbor1 + Neighbor2" to prevent staircases
            # "Neighbor1 != Neighbor2" to prevent sandwiches
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
    #Capacity bounds
    for z in range(1, MAX_COL_NUMBER+1):
        maxCount = get_max_capacity(z, GRID_SIZE)
        if maxCount is None:
            continue
        print(f"Bounding Color {z} to at most {maxCount} occurances")
        zPresenceBooleans = []

        for r in range(GRID_SIZE):
            for c in range(GRID_SIZE):
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
    print(len(model.Proto().constraints))
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print(f"SUCCESS: Solution Found for {GRID_SIZE}x{GRID_SIZE} Square Grid!")
        for r in range(GRID_SIZE):
            row = [solver.Value(grid[r, c]) for c in range(GRID_SIZE)]
            print(row)
    else:
        print("No solution found.")

def solve_finite():
    print(f"\n[{time.strftime('%H:%M:%S')}] Starting setup for Max Colors: {MAX_COL_NUMBER}")
    model = cp_model.CpModel()
    
    # 1. CREATE VARIABLES
    print(f"  > Creating variables for {GRID_SIZE}x{GRID_SIZE} grid...")
    grid = {}
    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            grid[r, c] = model.NewIntVar(1, MAX_COL_NUMBER, f'grid_{r}_{c}')

    # ---------------------------------------------------------
    # A. PACKING CONSTRAINTS (Finite Exclusion Zone)
    # ---------------------------------------------------------
    if PACKING:
        print("  > Building Packing Constraints...")
        for z in range(1, MAX_COL_NUMBER + 1):
            # Progress update every 10 colors to show activity
            if z % 10 == 0 or z == MAX_COL_NUMBER:
                print(f"    ... processing color {z}/{MAX_COL_NUMBER}")
                
            for r in range(GRID_SIZE):
                for c in range(GRID_SIZE):
                    # Switch variable: Is grid[r,c] == z?
                    b = model.NewBoolVar(f'is_{z}_{r}_{c}')
                    model.Add(grid[r, c] == z).OnlyEnforceIf(b)
                    model.Add(grid[r, c] != z).OnlyEnforceIf(b.Not())

                    # If 'b' is True, forbid 'z' in the exclusion zone
                    for dy in range(-z, z + 1):
                        for dx in range(-z, z + 1):
                            if dy == 0 and dx == 0: continue
                            
                            # Manhattan check
                            if abs(dy) + abs(dx) <= z:
                                nr, nc = r + dy, c + dx
                                
                                # BOUNDARY CHECK: Only apply if neighbor is inside grid
                                if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
                                    model.Add(grid[nr, nc] != z).OnlyEnforceIf(b)

    # ---------------------------------------------------------
    # C. VERTEX ADJACENCY RULES
    # ---------------------------------------------------------
    print("  > Building Vertex Adjacency & Arithmetic Rules...")
    neighbor_offsets = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    for r in range(GRID_SIZE):
        for c in range(GRID_SIZE):
            current = grid[r, c]
            
            # 1. Collect Valid Neighbors (Handle Corners/Edges)
            valid_neighbors = []
            for dy, dx in neighbor_offsets:
                nr, nc = r + dy, c + dx
                if 0 <= nr < GRID_SIZE and 0 <= nc < GRID_SIZE:
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
            # "2 * Current != Neighbor1 + Neighbor2"
            for i in range(len(valid_neighbors)):
                for j in range(i + 1, len(valid_neighbors)):
                    n1 = valid_neighbors[i]
                    n2 = valid_neighbors[j]
                    if STAIRS:
                        model.Add(2 * current != n1 + n2)
                    if SANDWICHES:
                        model.Add(n1!=n2)



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
        for r in range(GRID_SIZE):
            row = [solver.Value(grid[r, c]) for c in range(GRID_SIZE)]
            final_grid.append(row)
            print(f"   {row}")
        return True
    else:
        print(f"  >>> FAILURE: No solution found within constraints (Time: {elapsed:.2f}s).")
        return False

def get_max_capacity(z, grid_size):
    maxTorusDist = 2 * (grid_size // 2)
    if z >= maxTorusDist:
        return 1
    if 3 * (z + 1) > 2 * grid_size:
        return 2
    if 4 * (z + 1) > 2 * grid_size:
        return 3
    return None

if __name__ == "__main__":
    if TORODORIAL:
        solve_infinite()
    else:
        solve_finite()