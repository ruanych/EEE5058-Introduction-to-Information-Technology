def binary_search_left(arr, item):
    """
    same as np.searchsorted(arr, item, side='left')

    @param arr: a sorted array, ascending order
    @param item: the item to search

    @return: the index of the first element that is greater than or equal to item
    """
    low = 0
    high = len(arr) - 1
    while low <= high:
        mid = (low + high) // 2
        if arr[mid] >= item:
            high = mid - 1
        else:
            low = mid + 1
    return low


def binary_search_right(arr, item):
    """
    same as np.searchsorted(arr, item, side='right')

    @param arr: a sorted array, ascending order
    @param item: the item to search

    @return: the index of the first element that is greater than item
    """
    low = 0
    high = len(arr) - 1
    while low <= high:
        mid = (low + high) // 2
        if arr[mid] > item:
            high = mid - 1
        else:
            low = mid + 1
    return low
