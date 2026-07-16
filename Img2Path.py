import cv2
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation


def shape(arr):
    row = len(arr)
    col = len(arr[0])
    print(row, col)


def distance(p1, p2):
    return np.linalg.norm(p2 - p1)

def bilinear_sample(img, x, y): # find the gradient magnitude along the gradient angle

    x0 = int(np.floor(x))
    y0 = int(np.floor(y))

    x1 = x0 + 1
    y1 = y0 + 1

    if (
        x0 < 0 or x1 >= img.shape[1] or
        y0 < 0 or y1 >= img.shape[0]
    ):
        return 0

    fx = x - x0 # fractional distance
    fy = y - y0

    q11 = img[y0, x0] # top left
    q21 = img[y0, x1] # top right
    q12 = img[y1, x0] # bottom left
    q22 = img[y1, x1] # bottom right

    top = q11 * (1 - fx) + q21 * fx # top row 
    bottom = q12 * (1 - fx) + q22 * fx # bottom row

    return top * (1 - fy) + bottom * fy # vertically along the top to bottom column

def findOverlap(contours):

    idx = 0

    for contour in contours:
        dx = contour[1:,0,0] - contour[:-1,0,0]
        dy = contour[1:,0,1] - contour[:-1,0,1]
        dot = dx[:-1]*dx[1:] + dy[:-1]*dy[1:]
        for i in range(len(dot)):
            if dot[i] == -1:
                turn_idx = i
                contours.pop(idx)
                contour = contour[:turn_idx+1]
                contours.append(contour)
                break
        idx =+ 1


    return None

def Canny_detector(img, weak_th=None, strong_th=None): # Geeks for geeks with out approximating angles
    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # Grey scale
    
    img = cv2.GaussianBlur(img, (9, 9), 4) # Smoothen image, the more I smoothen the less detail

    gx = cv2.Sobel(
    np.float32(img),  # source image
    cv2.CV_64F,       # output data type
    1,                # x derivative order
    0,                # y derivative order
    3)                # kernel size nxn
    gy = cv2.Sobel(np.float32(img), cv2.CV_64F, 0, 1, 3)

    mag, ang = cv2.cartToPolar(gx, gy, angleInDegrees= False) # needs to be in degrees for g4g version

    height, width = img.shape

    mag_max = np.max(mag)
    if weak_th is None:
        weak_th = mag_max * 0.15
    if strong_th is None:
        strong_th = mag_max * 0.5

   
    nms = np.zeros_like(mag) # non-max suppression

    for i_x in range(1, width-1):
        for i_y in range(1, height-1):

            angle = ang[i_y, i_x]
            magnitude = mag[i_y, i_x]
            dx = np.cos(angle)
            dy = np.sin(angle)
            map_x_b = i_x - dx
            map_y_b = i_y - dy
            map_x_f = i_x + dx
            map_y_f = i_y + dy

            before = bilinear_sample(mag, map_x_b, map_y_b)
            after = bilinear_sample(mag, map_x_f, map_y_f)

            if magnitude >= before and magnitude >= after:
                nms[i_y, i_x] = magnitude
            else:
                nms[i_y, i_x] = 0

    result = np.zeros_like(nms)

    strong = 255
    weak = 150

    for i_x in range(width):
        for i_y in range(height):
            val = nms[i_y, i_x]

            if val >= strong_th:
                result[i_y, i_x] = strong
            elif val >= weak_th:
                result[i_y, i_x] = strong
            else:
                result[i_y, i_x] = 0

    return result

def Get_Contours(edges_img):
    contours, hierarchy = cv2.findContours(
    np.uint8(edges_img),
    cv2.RETR_LIST,
    cv2.CHAIN_APPROX_NONE)

    filtered_contours = []

    for contour in contours:
        # length = cv2.arcLength(contour, False)

        # if length > 0:
        filtered_contours.append(contour)
    
    filtered_contours = findOverlap(filtered_contours)

    # Connecting contours for a no pencil lift image

    remaining_contours = filtered_contours
    # print("THis is remaining contours")
    # print(remaining_contours)
    merged = remaining_contours.pop(0)
    current_point = merged[-1][0]
    bridges = []

    last_closest_point = []
    while (len(remaining_contours) > 0):

        print()
        print("New Loop: ")
        print()
        
        best_idx = None
        best_distance = float("inf")
        reverse_next = False
        
        print("This is the current point")
        print(current_point)
        print("This is the last closest point")
        print(last_closest_point)
        for i, contour in enumerate(remaining_contours):
            

            start = contour[0][0]
            end   = contour[-1][0]

            d_start = distance(current_point, start)
            d_end   = distance(current_point, end)

            if d_start < best_distance:
                best_distance = d_start
                best_idx = i
                reverse_next = False

            if d_end < best_distance:
                best_distance = d_end
                best_idx = i
                reverse_next = True
        print("Here are the distance, idx, and reverse:")
        print(best_distance, best_idx, reverse_next)

        next_contour = remaining_contours.pop(best_idx)
      
        
        if reverse_next:
            next_contour = next_contour[::-1]
       
        print("THis is the next contours, the one that is setting up to merge:")
        print(shape(next_contour))

        p1 = current_point
        p2 = next_contour[0][0]

        last_closest_point = next_contour[0][0]
        current_point = next_contour[-1][0]

        print("This is p1 and p2")
        print(p1,p2)

        num_points = int(np.linalg.norm(p2 - p1))

        xs = np.linspace(p1[0], p2[0], num_points)
        ys = np.linspace(p1[1], p2[1], num_points)

        bridge = np.array([[[int(x), int(y)]]
        for x, y in zip(xs, ys)],
        dtype=np.int32)

        print("This is the bridge before any post processing")
        print(bridge.shape)

        if bridge.shape == (0,) or len(bridge) < 6: # There should always be a distance between the two points or else they would be in the same contour
            bridge = np.empty((0, 1, 2), dtype=np.int32)

        merged = np.vstack([
        merged,
        bridge,
        next_contour
        ])

        bridges.append(bridge);
        print("In loop remaining contours:")
        try:
            shape(remaining_contours)
        except:
            None
        print("This was the bridge just created:")
        print(bridge.size)
  
    return bridges, merged, contours


def main():
    frame = cv2.imread('Sample.png')
    if frame is None:
        print("Error: image not found! Please check the path.")
        return
    
    img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    canny_img = Canny_detector(frame)
    print("This is canny img")
    print(canny_img)
    bridges, merged, contours = Get_Contours(canny_img)
    inv = cv2.bitwise_not(canny_img)
    x = merged[:,0,0]
    y = merged[:,0,1]

    print()
    print()

    print("This is contours:")
    shape(contours)
    print("This is merged:")
    shape(merged)
    print("This is bridges:")
    shape(bridges)
    # Plotting
    fig, axs = plt.subplots(2, 2)

    ax1 = axs[0,0]
    ax2 = axs[0,1]
    ax3 = axs[1,0]
    ax4 = axs[1,1]

    plt.title('Input Image')
    ax1.imshow(img)
    plt.axis('off')
    plt.title('Canny Edges')
    ax2.imshow(canny_img, cmap='gray')
    plt.axis('off')
    plt.title('Contours from Canny Edges')
    cv2.drawContours(img, contours, -1, (0, 255, 0), 1)
    ax3.imshow(img)
    plt.axis("off")

    plt.title('Bridges')

    def update(frame):

        ax4.clear()
        ax4.imshow(canny_img, cmap='gray')

        for bridge in bridges[:frame+1]:

            x = bridge[:,0,0]
            y = bridge[:,0,1]

            ax4.plot(x, y, color = 'red')
    ani = FuncAnimation(
    fig,
    update,
    frames=len(bridges),
    interval=1000,
    repeat = False
    )

    # cv2.drawContours(img, bridges, -1, (0, 0, 255), 1)

    plt.imshow(img)
    # plt.title('Merged edges')
    # plt.imshow(img, cmap='gray')
    # plt.plot(x, y, linewidth=1)
    plt.show()





    



if __name__ == "__main__":
    main()