import time
from CarSupport import Camera
c = Camera()

for i in range(100):
   img = c.take_picture()
   psi = c.compute_psi(DEBUG=True)
   time.sleep(0.1)

