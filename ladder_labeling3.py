# given an initial top path sequence of the ladder, 
# brute force the bottom path of the ladder, 
# taking the minimum vertex label available (greedy)

def main():
    # how many vertices to stop at
    stop_at_vertex = 800

    # top path vertex labeling
    #top_path_vertices = [6, 4, 1, 3, 7, 1, 5, 2]
    #top_path_vertices = [1, 5, 2,  6, 4,  1,  3, 7,]
    #top_path_vertices = [3, 7, 1, 5, 2, 6, 4, 1,]
    #top_path_vertices = [1, 3, 4, 1, 5, 2, 6]
    #top_path_vertices = [1, 4, 6, 2, 3, 7, 1, 4, 10, 2, 3, 11, 1, 4, 9, 2, 3, 12, 2, 7, 3, 5]
    top_path_vertices = [1, 3, 2, 5, 4, 6, 2, 3, 7, 2, 5, 3, 2, 6, 4, 3]

    give_up = False

    bottom_path_vertices = [find_valid_start_vertex(top_path_vertices)]

    while True:
        added_vertex = 1
        while True:
            if added_vertex > 40:
                # its not looking good, give up to prevent infinite loop
                give_up = True
                break

            bottom_path_vertices.append(added_vertex)

            # keep updating top_path repetition
            if len(top_path_vertices) <= len(bottom_path_vertices):
                top_path_vertices += top_path_vertices

            # print paths so we can see what's going on
            #print('    checking added vertex', added_vertex)
            #print_stuff(top_path, bottom_path, top_path_vertices, bottom_path_vertices)

            if check_if_new_labeling_is_valid(top_path_vertices, bottom_path_vertices):
                #print('* labeling is valid after adding vertex', added_vertex)
                added_vertex = 1
                break
            else:
                #print('*** labeling is not valid after adding vertex', added_vertex)
                # remove what we added and continue
                bottom_path_vertices = bottom_path_vertices[:-1]
                added_vertex += 1
                continue

        # let's look at it after some amount of vertices
        if len(bottom_path_vertices) == stop_at_vertex:
            print('stop_at_vertex =', stop_at_vertex)
            break

        if give_up:
            print('i >', added_vertex - 1, ', giving up')
            break

    print_stuff(top_path_vertices, bottom_path_vertices)

    find_immediate_repeat_min_k(bottom_path_vertices, k=20)


# finds the first valid start vertex label in the bottom path sequence
def find_valid_start_vertex(top_path_vertices):
    #print('finding first vertex in bottom path')

    first_vertex = 1

    while True:
        if check_if_new_labeling_is_valid(top_path_vertices, [first_vertex], is_first = True):
            #print('* labeling is valid with first vertex', first_vertex)
            return first_vertex
        else:
            #print('*** labeling is not valid with first vertex', first_vertex)
            first_vertex += 1
            continue


# return False if invalid, True if valid
# this only checks the new added vertex, not everything
def check_if_new_labeling_is_valid(top_vertices, bottom_vertices, is_first = False):
    # if we are figuring out first vertex, logic is different. checks with bottom path are not necessary
    if is_first:
        if (
            # check packing with top path
            check_packing_with_top_path(top_vertices, bottom_vertices) == False

            # check diff labeling with top path
            or check_difference_labeling_with_top_path(top_vertices, bottom_vertices) == False
        ):
            return False
        
        return True
    
    # --- we have added first vertex ---
    # if difference labeling or packing fails with this added vertex, return False
    if (
        # check difference labeling in bottom path
        check_difference_labeling_in_path(bottom_vertices) == False
        
        # check diff labeling with top path
        or check_difference_labeling_with_top_path(top_vertices, bottom_vertices) == False
        
        # check packing in bottom path
        or check_packing_in_path(bottom_vertices) == False

        # check packing with top path
        or check_packing_with_top_path(top_vertices, bottom_vertices) == False
    ):
        return False
    
    return True

def check_difference_labeling_in_path(path):
    #print(path)
    added_vert_index = len(path) - 1
    added_vert = path[-1]
    previous_vertex = path[added_vert_index - 1]
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
    for offset in range(1, added_vert + 1):
        check_index = added_vert_index - offset
        if check_index < 0:
            break
        if path[check_index] == added_vert:
            #print(check_index, path[check_index], added_vert, path)
            #print('packing in top path failed (backwards)')
            return False
    return True

def check_difference_labeling_with_top_path(top_verts, bot_verts):
    i = len(bot_verts) - 1
    added_vert = bot_verts[-1]
    # check diff labeling with top path
    # the index we are at (last index of bottom vertices)
    current_index = i
    # this is the vertex label directly above the vertex we added to bottom
    top_path_vertex_above = top_verts[current_index]
    #print(top_path_vertex_above)
    #print(len(bottom_verts) - 1)
    difference_of_top_and_bottom_vertex = abs(top_path_vertex_above - added_vert)
    
    top_path_vertex_above_left_edge = abs(top_verts[current_index] - top_verts[current_index - 1])
    top_path_vertex_above_right_edge = -1
    if current_index < len(top_verts) - 1:
        top_path_vertex_above_right_edge = abs(top_verts[current_index] - top_verts[current_index + 1])
    else:
        top_path_vertex_above_right_edge = abs(top_verts[current_index] - top_verts[0]) # loop around
    if (
        added_vert == top_path_vertex_above
        or added_vert == difference_of_top_and_bottom_vertex
        or difference_of_top_and_bottom_vertex == top_path_vertex_above
        or difference_of_top_and_bottom_vertex == top_path_vertex_above_left_edge
        or difference_of_top_and_bottom_vertex == top_path_vertex_above_right_edge
    ):
        #print('diff labeling with top path failed')
        return False
    
    return True

def check_packing_with_top_path(top_verts, bot_verts):
    i = len(bot_verts) - 1
    added_vert = bot_verts[-1]
    # check packing with top path
    # the index we are at (last index of bottom vertices)
    top_index_above = i

    for offset in range(0, added_vert):
        check_index = top_index_above - offset
        if check_index < 0:
            # wrap around to the end
            check_index = len(top_verts) + check_index
            # same TODO as below. edit: believe this is fixed
            if check_index < top_index_above: #0:
                break
        if top_verts[check_index] == added_vert:
            #print('packing in top path failed (backwards)')
            return False
        
    # 2) forward check from start to end
    for offset in range(0, added_vert):
        check_index = top_index_above + offset
        if check_index >= len(top_verts):
            # wrap around to the start
            check_index = check_index - len(top_verts)
            # TODO if wrapped to same part, break. this still goes to very end. edit: believe this is fixed
            #print(check_index)
            if check_index >= top_index_above: #len(top_verts):
                break
        if top_verts[check_index] == added_vert:
            #print('packing in top path failed (forward)')
            return False
    
    return True
        
def print_stuff(top_vertices, bottom_vertices):
    # print paths so we can see what's going on
    print('vertices of top and bot paths:')
    print('top:', top_vertices)
    print('bot:', bottom_vertices)


def find_immediate_repeat_min_k(lst, k=10):
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