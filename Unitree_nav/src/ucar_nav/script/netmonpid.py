import psutil

pid = 110998  # 进程ID
p = psutil.Process(pid)
print(p)
print(p.io_counters())
print(p.net_io_counters())
