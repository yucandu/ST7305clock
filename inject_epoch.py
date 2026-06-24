import time

Import("env")

epoch_time = int(time.mktime(time.localtime())) + 30

# Print a highly visible message to the terminal
print(f"\n---> SUCCESS: INJECTING BUILD_EPOCH = {epoch_time} <---\n")

env.Append(CPPDEFINES=[
    ("BUILD_EPOCH", epoch_time)
])