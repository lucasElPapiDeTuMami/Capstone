import time
print("hihi")
from CarSupport import Camera
c = Camera()

print("hi")
MAXSIZE = 100
psi = [0] * MAXSIZE
for i in range(100):
   img = c.take_picture()
   p = c.compute_psi(img,DEBUG=True)
   time.sleep(0.1)
   print(i)
   psi[i] = p

with open('PSI.txt','w') as f:
    for i in range(MAXSIZE):
        f.write(f"{psi[i]}")
        f.write('\n')
