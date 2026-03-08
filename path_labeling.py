# brute force infinite path given prohibitted numbers
# taking the minimum vertex label available (greedy)
# gives only upper bounds

import time  # added for timing

prohibitted_numbers = {1,2,3,4} # can't use these numbers (leave blank to allow all)
stop_at_vertex = 10000 # if program says can't find a repeat, try increasing this. 
# a full repetition check for ~10k vertices runs nearly instantly, for ~100k vertices takes a few seconds.

min_repeat_len = 6 # min length of repeated segment
max_repeat_len = stop_at_vertex // 2 # max length of repeated segment. 
# useful to cut down computation if we believe repeat len won't be higher than a certain number
# (will take min of this and path len // 2)

def main():
    start_time = time.time()  # start timer

    give_up = False

    path_vertices = [find_valid_start_vertex()]

    # store last appearance index for each vertex
    last_seen = {path_vertices[0]: 0}

    append_vertex = path_vertices.append
    pop_vertex = path_vertices.pop

    while True:
        added_vertex = 1

        while True:
            added_vertex = return_allowed_vertex_num(added_vertex)

            if added_vertex > 50:
                give_up = True
                break

            append_vertex(added_vertex)

            if check_if_new_labeling_is_valid(path_vertices, last_seen):
                last_seen[added_vertex] = len(path_vertices) - 1
                added_vertex = 1
                break
            else:
                pop_vertex()
                added_vertex += 1
                continue

        if len(path_vertices) == stop_at_vertex:
            print('stop_at_vertex =', stop_at_vertex)
            pd_time = time.time()
            print(f"Path done time: {pd_time - start_time:.2f} seconds")
            break

        if give_up:
            print('i >', added_vertex - 1, ', giving up')
            break

    #print(path_vertices) # let's see it
    print("Max of path:", max(path_vertices))
    find_immediate_repeat_min_k(path_vertices, k=min_repeat_len, max_L=max_repeat_len)

    end_time = time.time()  # end timer
    print(f"Total execution time: {end_time - start_time:.2f} seconds")


def return_allowed_vertex_num(num):
    prohibitted = prohibitted_numbers
    while num in prohibitted:
        num += 1
    return num


def find_valid_start_vertex():
    return return_allowed_vertex_num(1)


def check_if_new_labeling_is_valid(path_vertices, last_seen):
    if (
        check_difference_labeling_in_path(path_vertices) == False
        or check_packing_in_path(path_vertices, last_seen) == False
    ):
        return False
    return True


def check_difference_labeling_in_path(path):
    added_vert_index = len(path) - 1
    added_vert = path[added_vert_index]
    previous_vertex = path[added_vert_index - 1]

    added_edge = abs(added_vert - previous_vertex)

    if (
        added_vert == added_edge
        or added_vert == previous_vertex
        or added_edge == previous_vertex
    ):
        return False

    if added_vert_index >= 3:
        previous_previous_vertex = path[added_vert_index - 2]
        previous_edge = abs(previous_vertex - previous_previous_vertex)

        if added_edge == previous_edge:
            return False

    return True


def check_packing_in_path(path, last_seen):
    added_vert_index = len(path) - 1
    added_vert = path[added_vert_index]

    if added_vert in last_seen:
        if added_vert_index - last_seen[added_vert] <= added_vert:
            return False

    return True


import numpy as np

def find_immediate_repeat_min_k(lst, k=10, max_L=200):
    arr = np.asarray(lst, dtype=np.int32)
    n = len(arr)

    max_L = min(max_L, n // 2)

    for L in range(k, max_L + 1):
    #for L in range(max_L, k-1, -1): # reversed! since we believe the segment that repeats is long
        
        if L % 100 == 0:
            print("L:", L)

        eq = arr[:-L] == arr[L:]

        # find start/end of True runs
        padded = np.concatenate(([0], eq.view(np.int8), [0]))
        diff = np.diff(padded)

        starts = np.where(diff == 1)[0]
        ends = np.where(diff == -1)[0]

        lengths = ends - starts

        valid = np.where(lengths >= L)[0]
        if len(valid) > 0:
            start = starts[valid[0]]

            segment = arr[start:start+L]

            print("Repeating segment:", segment.tolist())
            print("Length:", L)
            print("First appears at index:", start)
            print("Repeats at index:", start + L)
            print("Max:", segment.max())

            return segment.tolist(), start, start + L

    print("No immediate repetition of length ≥", k)
    return None, None, None


main()
