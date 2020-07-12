import csv
with open('/home/pi/P0Wcrosshair/zero.csv') as f:
    data = dict(filter(None, csv.reader(f)))

if(len(data) != 14):
    print("Len: "+str(len(data))+" should be 14")
else:
    print("Length is 14!")

print("Raw Data")
print(data)
list = []

for key in data.keys():
    data[key] = int(data[key])
#    print data[key]

#if(len(data) != 14):
#    print("Len: "+str(len(data))+" should be 14")
#else:
#    print("Length is 14!")

data = {int(k) : v for k, v in data.items()}

print("In Order")
print(data)

for key in sorted(data.keys()):
#    print("%s: %s" % (key, data[key]))
    list.append(data[key])

print("Ordered list")
print(list)
