import csv
import matplotlib.pyplot as plt

with open("./check_lidar.log", 'r') as f:
    reader = csv.reader(f, delimiter=':')
    data_list = []
    
    for row in reader:
        # import pdb; pdb.set_trace()
        sec = row[2].split(' ')[0]
        # data_list.append(sec)
        print(sec)
# x = range(len(data_list))
# #plt.scatter(x, data_list,marker='o')
# plt.hist(data_list,bins=100)
# plt.savefig("3lidar.png")