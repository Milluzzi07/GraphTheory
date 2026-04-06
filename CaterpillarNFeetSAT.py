import multiprocessing
from concurrent.futures import ProcessPoolExecutor
from ortools.sat.python import cp_model
import time

# --- CONFIGURATION ---
MIN_WIDTH: int = 30
MAX_WIDTH: int = 30
MAX_COLOR: int = 17
NUM_FEET: int = 3
HEIGHT: int = NUM_FEET + 1
TIME_LIMIT: int = 1800
CPU_THREADS_USED: int = 16 #Set to 0 to use all available cores
MAX_MEMORY_IN_MB: int = 0 #Set to 0 to use all available memory
IS_FINITE: bool = True # Set to True for open boundaries, False for cylindrical

def get_flat_idx(r, c, width):
    return r * width + c

# --- WORKER FUNCTION ---
def generate_constraints_for_chunk(chunk_rows, width, HEIGHT, max_color, is_finite):
    """
    Generates constraints for a specific set of rows.
    Returns lists of:
      1. Adjacency pairs: (idx1, idx2) -> implies !=, no doubles
      2. Arithmetic triplets: (idx_curr, idx_n1, idx_n2)
    """
    adjacency_packets = []
    arithmetic_packets = []

    for r in chunk_rows:
        for c in range(width):
            curr_idx = get_flat_idx(r, c, width)
            
            # --- 2. ADJACENCY & ARITHMETIC ---
            # Determine immediate neighbors based on n-feet caterpillar topology
            immediate_neighbors = []
            
            if r == 0:
                # Spine row (r=0) connects horizontally to other spine nodes 
                if is_finite:
                    if c - 1 >= 0:
                        immediate_neighbors.append(get_flat_idx(0, c - 1, width)) # Left spine
                    if c + 1 < width:
                        immediate_neighbors.append(get_flat_idx(0, c + 1, width)) # Right spine
                else:
                    immediate_neighbors.append(get_flat_idx(0, (c - 1) % width, width)) # Left spine
                    immediate_neighbors.append(get_flat_idx(0, (c + 1) % width, width)) # Right spine
                
                # Connects to all feet on the same column
                for f in range(1, NUM_FEET + 1):
                    immediate_neighbors.append(get_flat_idx(f, c, width))               # Foot
            else:
                # Foot row only connects to the spine node in the same column
                immediate_neighbors.append(get_flat_idx(0, c, width))               
                
            for n_idx in immediate_neighbors:
                # Deduplication for Adjacency (A != B, etc)
                if n_idx > curr_idx:
                    adjacency_packets.append((curr_idx, n_idx))
            
            # Arithmetic (2*A != B + C)
            for i in range(len(immediate_neighbors)):
                for j in range(i + 1, len(immediate_neighbors)):
                    n1 = immediate_neighbors[i]
                    n2 = immediate_neighbors[j]
                    arithmetic_packets.append((curr_idx, n1, n2))

    return (adjacency_packets, arithmetic_packets)

def solve_nfeet_caterpillar_sat_parallel(width, max_color, time_limit, is_finite):
    model = cp_model.CpModel()
    start_time = time.time()
    
    # 1. CREATE VARIABLES (Main Thread)
    flat_vars: list[cp_model.IntVar] = []
    b_is_z = {}
    for r in range(HEIGHT):
        for c in range(width):
            var = model.NewIntVar(1, max_color, f'c_{r}_{c}')
            flat_vars.append(var)
            
            z_vars = []
            for z in range(1, max_color + 1):
                b = model.NewBoolVar(f'is_{z}_{r}_{c}')
                b_is_z[r, c, z] = b
                z_vars.append(b)
                model.Add(var == z).OnlyEnforceIf(b)
            model.AddExactlyOne(z_vars)
    
    # Grid wrapper for final output
    grid = {}
    for r in range(HEIGHT):
        for c in range(width):
            grid[r, c] = flat_vars[get_flat_idx(r, c, width)]

    # 2. PARALLEL GENERATION
    num_cores = multiprocessing.cpu_count()
    rows = list(range(HEIGHT))
    chunk_size = max(1, len(rows) // num_cores)
    row_chunks = [rows[i:i + chunk_size] for i in range(0, len(rows), chunk_size)]
    
    print(f"Generating constraints on {num_cores} cores...", end=" ")
    
    with ProcessPoolExecutor() as executor:
        futures = [executor.submit(generate_constraints_for_chunk, chunk, width, HEIGHT, max_color, is_finite) 
                   for chunk in row_chunks]
        
        for future in futures:
            adj_pack, arith_pack = future.result()
            
            # 3. APPLY CONSTRAINTS
            for idx1, idx2 in adj_pack:
                u, v = flat_vars[idx1], flat_vars[idx2]
                model.Add(u != v)
                model.Add(u != 2 * v)
                model.Add(v != 2 * u)

            for idx_c, idx_n1, idx_n2 in arith_pack:
                current = flat_vars[idx_c]
                n1 = flat_vars[idx_n1]
                n2 = flat_vars[idx_n2]
                model.Add(2 * current - n1 - n2 != 0)

    # 3.5 APPLY MAXIMAL CLIQUES & SELF-INTERFERENCE
    print(f"\nGenerating maximal cliques...", end=" ")
    clique_time = time.time()
    
    if not is_finite:
        for z in range(1, max_color + 1):
            req_dist = 2 if z == 1 else z
            for nr in range(HEIGHT):
                # dist from node to itself around cycle:
                # spine: width
                # Foot: width + 2
                cycle_dist = width + (0 if nr == 0 else 2)
                if req_dist >= cycle_dist:
                    for nc in range(width):
                        model.Add(b_is_z[nr, nc, z] == 0)

    # Maximal Cliques (Packing)
    spine_r = 0
    for z in range(1, max_color + 1):
        req_dist = 2 if z == 1 else z
        k = req_dist // 2
        is_even = (req_dist % 2 == 0)
        
        for c in range(width):
            if not is_even:
                if is_finite and c + 1 >= width:
                    continue
                c_next = (c + 1) % width
            
            clique = []
            for nr in range(HEIGHT):
                for nc in range(width):
                    if is_finite:
                        dx1 = abs(nc - c)
                    else:
                        dx1 = min(abs(nc - c), width - abs(nc - c))
                    
                    dist_to_spine_nr = 1 if nr > 0 else 0
                    dist1 = dx1 + dist_to_spine_nr
                    
                    if is_even:
                        if dist1 <= k:
                            clique.append(b_is_z[nr, nc, z])
                    else:
                        if is_finite:
                            dx2 = abs(nc - c_next)
                        else:
                            dx2 = min(abs(nc - c_next), width - abs(nc - c_next))
                        dist2 = dx2 + dist_to_spine_nr
                        
                        if min(dist1, dist2) <= k:
                            clique.append(b_is_z[nr, nc, z])
            if len(clique) > 1:
                model.AddAtMostOne(clique)
    
    print(f"done in {time.time() - clique_time:.2f}s")
    
    # 3.6 SYMMETRY BREAKING
    # Force foot 1 < foot 2 < ... < foot N locally on each spine node vertically to break permutations
    if NUM_FEET > 1:
        for c in range(width):
            for f in range(1, NUM_FEET):
                model.Add(grid[f, c] < grid[f + 1, c])
        
    gen_time = time.time() - start_time
    print(f"Total generation done in {gen_time:.2f}s")

    # 4. SOLVE
    solver = cp_model.CpSolver()
    solver.parameters.max_time_in_seconds = time_limit
    solver.parameters.num_search_workers = CPU_THREADS_USED 
    solver.parameters.random_seed = 42
    solver.parameters.log_search_progress = True
    solver.parameters.max_memory_in_mb = MAX_MEMORY_IN_MB
    solve_start_time = time.time()
    status = solver.Solve(model)
    print(f"Solve time: {time.time() - solve_start_time:.2f}s")
    
    return status, solver, grid

def main():
    print(f"--- STARTING SEARCH ---")
    print(f"Spine length (Width): {MIN_WIDTH} - {MAX_WIDTH}")
    print(f"Feet per spine node:  {NUM_FEET}")
    print("-" * 30)

    for w in range(MIN_WIDTH, MAX_WIDTH + 1):
        print(f"Testing Width {w}...", end=" ", flush=True)
        result_status, solver, grid = solve_nfeet_caterpillar_sat_parallel(w, MAX_COLOR, TIME_LIMIT, IS_FINITE)
        
        if result_status == cp_model.OPTIMAL or result_status == cp_model.FEASIBLE:
            print(f"SUCCESS!")
            # Optional: Print solution
            for r in range(HEIGHT):
                print([solver.Value(grid[r, c]) for c in range(w)])
            break 
        else:
            print(f"NO SOLUTION")

if __name__ == "__main__":
    multiprocessing.freeze_support() # Good practice for Windows
    main()
