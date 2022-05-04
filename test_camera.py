import time
print("hihi")
from CarSupport import Camera
c = Camera()

print("hi")
for i in range(100):
   img = c.take_picture()
   psi = c.compute_psi(img,DEBUG=True)
   time.sleep(0.1)
   print(i)

