Drone-relative-positioning
==============================
Blog post: [https://matthew-bird.com/blogs/Drone-Rel-Pos.html](https://matthew-bird.com/blogs/Drone-Rel-Pos.html)

### Youtube videos
[![Demo Video 1](https://img.youtube.com/vi/QoqZZ-jRLvM/0.jpg)](https://www.youtube.com/watch?v=QoqZZ-jRLvM)

## Premise
My plan for this project was to determine the relative positioning and orientation of two or more objects with cameras using minimal usage of external libraries. I was inspired to do this by drone shows in HK.  

## How it works
### Overview
The code uses two cameras on the central drone to create a 3d map of features, then the other drones do the reverse from any two points to determine its own position in this 3d map of points. 

### Feature detection
To detect features, we use the [FAST algorithm](https://en.m.wikipedia.org/wiki/Features_from_accelerated_segment_test) to detect corners. To improve computation time, we also use some methods to remove ‘low quality’ key points. 

The first method, inspired by the SIFT algorithm, is to cull any points that aren’t substantially darker or lighter than the neighborhood. The second method, also inspired by the SIFT algorithm, is to cull any edges, which the FAST algorithm is often susceptible to classifying as corners. We do this by taking three (angle) ranges around the key point centered on the orientation of the key point. For each range, we find the average difference in brightness of the points in the range and the key point. If the value calculated for the central range is too close to either of the outside ranges, then we consider it an edge and cull it. 

<ins>Edge culling algorithm</ins>

<img width="600" alt="IMG_0333" src="https://github.com/user-attachments/assets/c6704a98-cddb-4db9-8a7f-587e61bc1438">

<ins>Img after FAST</ins>

![1](https://github.com/user-attachments/assets/5b1746d7-7412-4956-80d5-b7c74172db01)

<ins>Img after intensity cull</ins>

![2](https://github.com/user-attachments/assets/a5e6f0da-f2c0-4abf-a061-169a7c23ce8f)

<ins>Img after edge cull</ins>

![3](https://github.com/user-attachments/assets/4d571eab-a51a-486b-9c0c-d926e3a6f958)

With these key points, we calculate descriptors for each of these points that are scale and rotationally invariant and match the points to each other. 

My first attempt was to use the following two descriptors for each key point. However, these 3 descriptors alone weren’t enough to reliably match each key point. 

<img width="600" alt="IMG_0333" src="https://github.com/user-attachments/assets/c1dbc2a9-9376-4050-8348-33e8c13b3c36">

The method that ended up working was a bit more convoluted. First, we find the orientation of the key point through taking all nearby pixels in the point’s neighborhood whose brightness was within a certain range, then taking the average angle of the vectors from the key point to each pixel. 

With the orientation, we then split the area around the point into chunks while also taking into account orientation, a method inspired by the SIFT and SURF algorithms. This is also what gives us rotational invariance. Since the rotated points’ coordinates aren’t integers, we use 2d linear interpolation to approximate these values. 

<ins>Key point orientations</ins>

![4](https://github.com/user-attachments/assets/49dacaf3-3ea0-4b90-8481-96c953075ba1)

<ins>Descriptor chunks</ins>

<img width="400" alt="IMG_0333" src="https://github.com/user-attachments/assets/498bd97c-8120-420e-959c-f4d0343e7057">

After this, we create a vector for each square subsection in which we average all of the vectors starting from the center of the square to each point, and store the magnitude and direction. We also store the magnitude of the sum of the absolute value of each vector for an extra descriptor. 

<ins>Descriptor calculation</ins>

<img width="600" alt="IMG_0333" src="https://github.com/user-attachments/assets/11d71ecf-de00-4504-9517-9d7b08b83771">

With each key point now assigned a descriptor, we match key points across two images by taking the MSE of each key point in the first image to every other key point in the second image, and iteratively take the minimum value to match key points. 

<ins>Matched key points</ins>

![7](https://github.com/user-attachments/assets/b480903d-e2a0-4b23-9415-c49bba39c788)

### Feature triangulation
To find the 3d position of the features, we draw a line in 3d space from the center of both cameras on the central drone and approximate the intersecting point in 3d space to find the feature’s position in 3d space. 

<img width="300" alt="IMG_0333" src="https://github.com/user-attachments/assets/155d5324-84ad-4f4a-ac01-163bd41491f1">

### Reverse triangulation
To find the 3d position of a camera from any two features, we use the same method as before, but instead of drawing from the center of both cameras, we anchor the lines on the feature points in 3d space and approximate the intersection to find our camera’s position. We do this for every set of two features and remove outliers to get our final position. 

### Accounting for orientation
To account for the orientation, we first need to find our orientation. To find pitch and roll, I used an ADXL345 accelerometer. 

To find the yaw, we take any pair of matched features across two cameras, one from the central drone and another from the second drone. Then, we use the offset in angle of the key points to give us the relative yaw of every drone with respect to the central drone.

To account for yaw, we rotate each key point around the center of the image by the calculated yaw offset. 

<ins>Accounting for yaw (Note that the shape is replicated in the second image)</ins>
![8](https://github.com/user-attachments/assets/4e2cbcf5-5b1f-4cbe-9314-b93b34a420a1)

To account for pitch and roll on the central drone, after triangulating the 3d position of all the features, we then rotate their positions around the origin in accordance with the pitch and yaw to properly account for the pitch and roll.

<img width="600" alt="IMG_0333" src="https://github.com/user-attachments/assets/aa8a1c5a-11f1-42dd-a03b-a0f5d48b0153">

For the secondary drones, we rotate the lines we draw before we reverse triangulate the camera to counteract the pitch and roll of the camera.

<img width="300" alt="IMG_0333" src="https://github.com/user-attachments/assets/747223cb-1a1c-4691-8f25-9f6d89d25dc7">

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

The frames holding it all together are printed from [these .STL files](https://drive.google.com/drive/folders/1yhmFWAC9WZl5KNo4NMfis7nEZxAkWpuL?usp=sharing)(with paper as spacing to keep camera and accelerometer flat and taping the camera lenses to the body as they tend to flop around). I decided against implementing the project on real drones as I would have to make my own to avoid the high costs, they introduce many problems like pid tuning and make iterating take longer and more difficult. 

## Results
Overall, the project seemed to work pretty well. Compared to other systems, it’s a bit slow and not the most accurate, and it also doesn’t have the greatest tolerance for orientation differences, but I still think it’s quite impressive for its simplicity and cost, as well as the low quality RasPi cameras. 


In the future, it could be possible to find the 3d position of more features from any 2 drones so that the range is not limited to the central drone. The current system is also centralized, but it shouldn’t be too hard to fully decentralize it with more powerful raspberry pi computers. 
