# brute force path given prohibitted numbers
# taking the minimum vertex label available (greedy)
prohibitted_numbers = [1, 2, 3]
#prohibitted_numbers = [1, 3, 5, 7, 9]

def main():
    # how many vertices to stop at
    stop_at_vertex = 800

    give_up = False

    path_vertices = [find_valid_start_vertex()]

    while True:
        added_vertex = 1
        while True:
            added_vertex = return_allowed_vertex_num(added_vertex)

            if added_vertex > 40:
                # its not looking good, give up to prevent infinite loop
                give_up = True
                break

            path_vertices.append(added_vertex)

            if check_if_new_labeling_is_valid(path_vertices):
                #print('* labeling is valid after adding vertex', added_vertex)
                added_vertex = 1
                break
            else:
                #print('*** labeling is not valid after adding vertex', added_vertex)
                # remove what we added and continue
                path_vertices = path_vertices[:-1]
                added_vertex += 1
                continue

        # let's look at it after some amount of vertices
        if len(path_vertices) == stop_at_vertex:
            print('stop_at_vertex =', stop_at_vertex)
            break

        if give_up:
            print('i >', added_vertex - 1, ', giving up')
            break

    find_immediate_repeat_min_k(path_vertices, k=20)

def return_allowed_vertex_num(num):
    while num in prohibitted_numbers:
        num += 1
    return num


# finds the first valid start vertex label in the bottom path sequence
def find_valid_start_vertex():
    #print('finding first vertex in path')
    return return_allowed_vertex_num(1)


# return False if invalid, True if valid
# this only checks the new added vertex, not everything
def check_if_new_labeling_is_valid(path_vertices):
    # if difference labeling or packing fails with this added vertex, return False
    if (
        # check difference labeling in path
        check_difference_labeling_in_path(path_vertices) == False
        
        # check packing in path
        or check_packing_in_path(path_vertices) == False
    ):
        return False
    
    return True

def check_difference_labeling_in_path(path):
    added_vert_index = len(path) - 1
    added_vert = path[-1]
    #print(path)
    previous_vertex = path[-2]
    added_edge = abs(added_vert - previous_vertex)

    # check difference labeling in bottom path
    if (
        added_vert == added_edge
        or added_vert == previous_vertex
        or added_edge == previous_vertex
    ):
        #print('diff labeling in path failed')
        return False
    if added_vert_index >= 3: # if we have previous edge do the extra check
        # pp_vertex -- p_edge -- p_vertex -- added_edge -- added_vertex
        previous_previous_vertex = path[added_vert_index - 2]
        previous_edge = abs(previous_vertex - previous_previous_vertex)
        if added_edge == previous_edge:
            #print('diff labeling in path failed')
            return False
        
    return True

def check_packing_in_path(path):
    added_vert_index = len(path) - 1
    added_vert = path[-1]
    # staircase check for caterpillar
    # prev_vert = path[added_vert_index - 1]
    # prohibitted_combos = [[3, 4], [3, 5], [4, 6], [4, 7], [5, 8], [5, 9], [6, 10], [6, 11], [7, 12], [7, 13]]
    # for combo in prohibitted_combos:
    #     if [added_vert, prev_vert] == combo or [prev_vert, added_vert] == combo:
    #         return False

    for offset in range(1, added_vert + 1):
        check_index = added_vert_index - offset
        if check_index < 0:
            break # just break
        if path[check_index] == added_vert:
            #print(check_index, path[check_index], added_vert, path)
            #print('packing in top path failed (backwards)')
            return False
        
    return True


def find_immediate_repeat_min_k(lst, k=10):
    print(lst)
    n = len(lst)

    for i in range(n):
        max_L = (n - i) // 2
        for L in range(k, max_L + 1):
            if lst[i:i+L] == lst[i+L:i+2*L]:
                print("Repeating segment:", lst[i:i+L])
                print("Length:", L)
                print("First appears at index:", i)
                print("Repeats at index:", i + L)
                print("Max:", max(lst[i:i+L]))
                return lst[i:i+L], i, i + L
    
    print("No immediate repetition of length ≥", k)
    return None, None, None


main()