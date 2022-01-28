import diamond_finder


def autograder_diamond():

    with open("lab1_gt.txt", "r") as fh:
        ground_truths = eval(fh.read())

    predictions = diamond_finder.find_diamond()

    correct_num_diamonds = 0
    for img_filename in ground_truths:
        predicted_coords = predictions[img_filename]
        ground_truth_coords = ground_truths[img_filename]

        if len(predicted_coords) == len(ground_truth_coords):
            correct_num_diamonds += 1

    accuracy = correct_num_diamonds * 1.0 / len(ground_truths)
    return accuracy


if __name__ == "__main__":
    accuracy = autograder_diamond()
    print(accuracy)
