
total = 0
num = 0
mini = 100
maxi = 0

with open("TRItimes.txt", "r") as f:
    line = f.readline()
    times = line.split(",")[:-1]
    for t in range(len(times)):
        times[t] = times[t].strip()
        print(times[t])
        val = float(times[t])
        total += val
        if val < mini:
            mini = val
        if val > maxi:
            maxi = val
        num += 1

print("Tri Number: ", num)
print("Tri Avg Time: ", total/num)
print("Tri Min: ", mini, "  Tri Max: ", maxi)

total = 0
num = 0
mini = 100
maxi = 0
with open("CCDtimes.txt", "r") as f:
    line = f.readline()
    times = line.split(",")[:-1]
    for t in range(len(times)):
        times[t] = times[t].strip()
        val = float(times[t])
        total += val
        if val < mini:
            mini = val
        if val > maxi:
            maxi = val
        num += 1


print("CCD Number: ", num)
print("CCD Avg Time: ", total/num)
print("CCD Min: ", mini, "  CCD Max: ", maxi)
