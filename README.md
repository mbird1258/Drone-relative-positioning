Drone-relative-positioning
==============================
## Premise
My plan for this project was to determine the relative positioning and orientation of two or more objects with cameras using minimal usage of external libraries. I was inspired to do this by drone shows in HK. 

## How it works
### Overview
The code uses two cameras on the central drone to create a 3d map of features, then the other drones can do the reverse from any two points to determine its own position in this 3d map of points. 

### Feature detection
To detect features, we use the [FAST algorithm](https://en.m.wikipedia.org/wiki/Features_from_accelerated_segment_test) to detect corners. Then, we calculate descriptors for each of these points that are scale and rotationally invariant and match the points to each other. 


The descriptors used to match the points are 

### Feature triangulation
To find the 3d position of the features, we can draw a line in 3d space from the center of both cameras on the central drone and approximate the intersecting point in 3d space to find the feature’s position in 3d space. 

[insert image here of viewrect and lines intersection]

### Reverse triangulation
To find the 3d position of a camera from any two features, we can use the same method as before, but instead of drawing from the center of both cameras, we anchor the lines on the feature points in 3d space and approximate the intersection to find our camera’s position. We do this for every set of two features and remove outliers to get our final position. 

### Hardware
The full list of hardware used in this project are 3 RasPi 0Ws, 3 RasPi camera modules, 2 accelerometers, 2 power banks, 3 usb to micro usb cables, 3 micro SD cards, and a micro SD to usb adapter. 

Full list:
- [RasPi 0W](https://www.aliexpress.com/item/1005005792181612.html?spm=a2g0o.order_list.order_list_main.30.60651802iXxuYL)
- [RasPi camera module](https://www.aliexpress.com/item/32901067278.html?spm=a2g0o.order_list.order_list_main.25.60651802iXxuYL)
- [ADXL345 Accelerometer](https://www.aliexpress.com/item/32452794842.html?spm=a2g0o.order_list.order_list_main.20.60651802iXxuYL)
- [Power Bank](https://www.aliexpress.com/item/32974708727.html?spm=a2g0o.order_list.order_list_main.10.60651802iXxuYL)
- [MicroUSB Cable](https://www.aliexpress.com/item/32391749504.html?spm=a2g0o.order_list.order_list_main.15.60651802iXxuYL)
- [MicroSD](https://www.amazon.com/SanDisk-2-Pack-microSDHC-Memory-2x32GB/dp/B08GY9NYRM/ref=sr_1_3?crid=1O6LZJGU106Q9&dib=eyJ2IjoiMSJ9.tk0UAe6rKAf0FakbuJisoKUomV5T1j37HI71I94y2M_0QZwoxj-Tbw4sowKCr5WH9cyxWaNy7Mp6M_TIFeIaR_qOvvHAY7o7dNKHDUPhbLF3upGURhtAnm_L4jIt9CVhJRwHXjG2nIccV6KGlFkV8OSFyigdYplNKJ5PTfbVfDw2Fj8cdMeZttrEGsuu9y9oyI03ARWcVrcQE0bjQ0P35HTuzZoyZXaxSIMG2Q2Lq6c.5OWHbjcGPRzyfdBfUICGkGpP_jsD--OYMbEoxqXcZuc&dib_tag=se&keywords=32%2Bgb%2Bmicro%2Bsd%2Bcard&qid=1721202707&sprefix=32%2Bgb%2Bmicro%2Bsd%2Bca%2Caps%2C342&sr=8-3&th=1)
- [MicroSD Reader](https://www.aliexpress.com/item/1005005492821617.html?spm=a2g0o.order_list.order_list_main.5.60651802iXxuYL)

The Raspberry Pis are used with Socket to connect to the computer and send over the camera and accelerometer data. ([RasPi setup](https://docs.google.com/document/d/1zvPyD8OOzXOKGc8JJwU0GeqYuUTja-vIca7p4PPr_i4/edit?usp=sharing))

The frames holding it all together are printed from [these .STL files](https://drive.google.com/drive/folders/1yhmFWAC9WZl5KNo4NMfis7nEZxAkWpuL?usp=sharing). I decided against implementing the project on real drones as I would have to make my own to avoid the high costs, they introduce many problems like pid tuning and make iterating take longer and more difficult. Plus, they wouldn’t add much to the project. 

### Accounting for orientation
asdf

## Results
asdf

In the future, it could be possible to find the 3d position of more features from any 2 drones so that the range is not limited to the central drone. 
