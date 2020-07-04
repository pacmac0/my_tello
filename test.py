import concurrent.futures


def process(file, num1, num2):
    return [file, num1, num2]

# Hanging I dont know why 
gray = "test"
with concurrent.futures.ProcessPoolExecutor() as executor:
    p1 = executor.submit(process, gray, 1.5, 2)
    p2 = executor.submit(process, gray, 1.5, 2)
    p3 = executor.submit(process, gray, 1.5, 2)

    print(p1.result())
    print(p2.result())
    print(p3.result())

