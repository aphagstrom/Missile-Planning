import matplotlib.pyplot as plt


def plot_dubins(i, target_pos, p_s, p_e, crs, cls, cre, cle, w1, w2, w3, q1, q3, R):
    fig, ax = plt.subplots()
    ax.set_aspect('equal')
    
    plt.suptitle(''.join(["Missile ", str(i)]))
    
    # plot circle around target
    term_circle = plt.Circle((target_pos[1, 0], target_pos[0, 0]), 16500, fill = False, color = 'b') 
    ax.add_patch(term_circle)
    
    plt.scatter(p_s[1, 0], p_s[0, 0], color = 'g') # start position
    plt.scatter(p_e[1, 0], p_e[0, 0], color = 'r') # where missile intersects term_circle
    
    # plot the 4 Dubins circles
    plt.scatter(crs[1, 0], crs[0, 0], color = 'c')
    plt.scatter(cls[1, 0], cls[0, 0], color = 'c')
    plt.scatter(cre[1, 0], cre[0, 0], color = 'c')
    plt.scatter(cle[1, 0], cle[0, 0], color = 'c')
    circle_rs = plt.Circle((crs[1, 0], crs[0, 0]), R, fill = False, color = 'c')
    circle_ls = plt.Circle((cls[1, 0], cls[0, 0]), R, fill = False, color = 'c')
    circle_re = plt.Circle((cre[1, 0], cre[0, 0]), R, fill = False, color = 'c')
    circle_le = plt.Circle((cle[1, 0], cle[0, 0]), R, fill = False, color = 'c')
    ax.add_patch(circle_rs)
    ax.add_patch(circle_ls)
    ax.add_patch(circle_re)
    ax.add_patch(circle_le)
    
    # plot the straight line path
    plt.scatter(w1[1, 0], w1[0, 0], color = 'purple')
    x1 = w1[1, 0] - (w1[1, 0] - w2[1, 0])
    x2 = w1[1, 0]
    slope = q1[0, 0]/ q1[1, 0]
    y1 = w1[0, 0] - (w1[1, 0] - w2[1, 0])*slope
    y2 = w1[0, 0] 
    plt.plot([x1, x2], [y1, y2], color = 'purple')
    plt.scatter(w2[1, 0], w2[0, 0], color = 'purple')
    
    plt.autoscale()
    plt.show(block = False)