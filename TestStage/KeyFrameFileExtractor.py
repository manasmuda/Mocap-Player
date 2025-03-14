import random

def remove_lines_with_words(file_path, words_to_remove):
    """
    Reads a file and removes all lines that contain any of the specified words,
    ensuring that at least one occurrence is kept every 20-30 times per word.
    
    :param file_path: Path to the file.
    :param words_to_remove: A list of words. If a line contains any of these words,
                            it will be removed except for one in every 20-30 occurrences.
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as file:
            lines = file.readlines()
        
        word_counters = {word: 0 for word in words_to_remove}
        word_limits = {word: random.randint(30, 40) for word in words_to_remove}  # Random limit per word
        
        def should_remove(line):
            for word in words_to_remove:
                if word in line:
                    word_counters[word] += 1
                    if word_counters[word] >= word_limits[word]:
                        word_counters[word] = 0  # Reset counter after keeping one
                        word_limits[word] = random.randint(20, 30)  # Set new random limit
                        return False  # Keep this occurrence
                    return True  # Otherwise, remove it
            return False
        
        filtered_lines = [line for line in lines if not should_remove(line)]
        
        with open(file_path, 'w', encoding='utf-8') as file:
            file.writelines(filtered_lines)
        
        print("Lines containing specified words have been removed, ensuring one is kept every few occurrences.")
    except FileNotFoundError:
        print("Error: File not found.")
    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage
file_path = "martial_arts_test.amc"  # Change this to your file path
words_to_remove = ["rfemur", "rtibia", "rfoot", "rtoes", "lfemur", "ltibia", "lfoot", "ltoes", "rclavicle","rhumerus","rradius", "rwrist", "rhand","rfingers","rthumb", "lclavicle","lhumerus","lradius", "lwrist", "lhand","lfingers","lthumb"]  # Add words to remove
remove_lines_with_words(file_path, words_to_remove)
