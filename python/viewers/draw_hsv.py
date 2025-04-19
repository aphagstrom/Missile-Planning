"""
draw hypersonic vehicle
    - Update history:
        5/15/23 - RWB
"""
import numpy as np
import pyqtgraph.opengl as gl


class DrawHSV:
    def __init__(self, state, window):
        # get points that define the non-rotated, non-translated mav and the mesh colors
        self.hsv_points, self.hsv_meshColors = self.__get_points()
        rotated_points = self.__rotate_points(self.hsv_points, state.R)
        translated_points = self.__translate_points(rotated_points, state.pos.reshape((len(state.pos), 1)))
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.__points_to_mesh(translated_points)
        self.hsv_body = gl.GLMeshItem(vertexes=mesh,  # defines the triangular mesh (Nx3x3)
                                      vertexColors=self.hsv_meshColors,  # defines mesh colors (Nx1)
                                      drawEdges=True,  # draw edges between mesh elements
                                      smooth=False,  # speeds up rendering
                                      computeNormals=False)  # speeds up rendering
        #self.mav_body.setGLOptions('translucent')
        # ============= options include
        # opaque        Enables depth testing and disables blending
        # translucent   Enables depth testing and blending
        #               Elements must be drawn sorted back-to-front for
        #               translucency to work correctly.
        # additive      Disables depth testing, enables blending.
        #               Colors are added together, so sorting is not required.
        # ============= ======================================================
        window.addItem(self.hsv_body)  # add body to plot

    def update(self, state):
        # rotate and translate points defining mav
        rotated_points = self.__rotate_points(self.hsv_points, state.R)
        translated_points = self.__translate_points(rotated_points, state.pos.reshape((len(state.pos), 1)))
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.__points_to_mesh(translated_points)
        # draw MAV by resetting mesh using rotated and translated points
        self.hsv_body.setMeshData(vertexes=mesh, vertexColors=self.hsv_meshColors)

    def __rotate_points(self, points, R):
        "Rotate points by the rotation matrix R"
        rotated_points = R @ points
        return rotated_points

    def __translate_points(self, points, translation):
        "Translate points by the vector translation"
        translated_points = points + np.dot(translation, np.ones([1, points.shape[1]]))
        return translated_points

    def __get_points(self):
        """"
            Points that define the mav, and the colors of the triangular mesh
            Define the points on the aircraft following diagram in Figure C.3
        """
        # define hypersonic body parameters
        unit_length = 100.0
        fuse_l1 = 11.0 * unit_length
        fuse_l2 = 6.0 * unit_length
        fuse_l3 = -6.0 * unit_length
        fuse_l4 = -6.5 * unit_length
        fuse_l5 = -9.5 * unit_length
        fuse_l6 = -10.0 * unit_length
        fuse_l7 = -12.0 * unit_length
        fuse_w2 = 2.0 * unit_length
        fuse_h2 = 1.0 * unit_length
        fuse_w3 = 2.0 * unit_length
        fuse_h3 = 1.0 * unit_length
        fuse_w4 = 2.0 * unit_length
        fuse_h4 = 1.0 * unit_length
        fuse_w5 = 2.0 * unit_length
        fuse_h5 = 1.0 * unit_length
        fuse_w6 = 2.0 * unit_length
        fuse_h6 = 1.0 * unit_length
        fin_w = 3.0 * unit_length
        fin_l1 = 2.0 * unit_length
        fin_l2 = 4.0 * unit_length
        
        # points are in NED coordinates
        points = np.array([ [fuse_l1, 0.0, 0.0],  # point 0
                            [fuse_l2, fuse_w2, -fuse_h2],  # point 1
                            [fuse_l2, -fuse_w2, -fuse_h2],  # point 2 
                            [fuse_l2, -fuse_w2, fuse_h2],  # point 3 
                            [fuse_l2, fuse_w2, fuse_h2],  # point 4
                            [fuse_l3, fuse_w3, -fuse_h3],  # point 5
                            [fuse_l3, -fuse_w3, -fuse_h3],  # point 6 
                            [fuse_l3, -fuse_w3, fuse_h3],  # point 7
                            [fuse_l3, fuse_w3, fuse_h3],  # point 8
                            [fuse_l4, fuse_w4, -fuse_h4],  # point 9
                            [fuse_l4, -fuse_w4, -fuse_h4],  # point 10 
                            [fuse_l4, -fuse_w4, fuse_h4],  # point 11
                            [fuse_l4, fuse_w4, fuse_h4],  # point 12
                            [fuse_l5, fuse_w5, -fuse_h5],  # point 13
                            [fuse_l5, -fuse_w5, -fuse_h5],  # point 14 
                            [fuse_l5, -fuse_w5, fuse_h5],  # point 15
                            [fuse_l5, fuse_w5, fuse_h5],  # point 16
                            [fuse_l6, fuse_w6, -fuse_h6],  # point 17
                            [fuse_l6, -fuse_w6, -fuse_h6],  # point 18 
                            [fuse_l6, -fuse_w6, fuse_h6],  # point 19
                            [fuse_l6, fuse_w6, fuse_h6],  # point 20
                            [fuse_l7, 0.0, 0.0],  # point 21
                            # top fin
                            [fuse_l4, 0.0, -fuse_h4],  # point 22
                            [fuse_l4-fin_l1, 0.0, -fuse_h4-fin_w],  # point 23
                            [fuse_l4-fin_l2, 0.0, -fuse_h5-fin_w],  # point 24 
                            [fuse_l5, 0.0, -fuse_h5],  # point 25
                            # bottom fin
                            [fuse_l4, 0.0, fuse_h4],  # point 26
                            [fuse_l4-fin_l1, 0.0, fuse_h4+fin_w],  # point 27
                            [fuse_l4-fin_l2, 0.0, fuse_h5+fin_w],  # point 28 
                            [fuse_l5, 0.0, fuse_h5],  # point 29
                            # right fin
                            [fuse_l4, fuse_w4, 0.0],  # point 30
                            [fuse_l4-fin_l1, fuse_w4+fin_w, 0.0],  # point 31
                            [fuse_l4-fin_l2, fuse_w5+fin_w, 0.0],  # point 32 
                            [fuse_l5, fuse_w5, 0.0],  # point 33
                            # left fin
                            [fuse_l4, -fuse_w4, 0.0],  # point 34
                            [fuse_l4-fin_l1, -fuse_w4-fin_w, 0.0],  # point 35
                            [fuse_l4-fin_l2, -fuse_w5-fin_w, 0.0],  # point 36 
                            [fuse_l5, -fuse_w5, 0.0],  # point 37
                           ]).T

        #   define the colors for each face of triangular mesh
        red = np.array([1., 0., 0., 1])
        green = np.array([0., 1., 0., 1])
        blue = np.array([0., 0., 1., 1])
        yellow = np.array([1., 1., 0., 1])
        # colors
        mygrey1 = np.array([0.8, 0.8, 0.8, 1])  # light
        mygrey2 = np.array([0.6, 0.6, 0.6, 1])
        mygrey3 = np.array([0.5, 0.5, 0.5, 1])
        mygrey4 = np.array([0.3, 0.3, 0.3, 1])  # dark

        meshColors = np.empty((32, 3, 4), dtype=np.float32)
        meshColors[0] = mygrey1  # nose-top
        meshColors[1] = mygrey1  # nose-right
        meshColors[2] = yellow  # nose-bottom
        meshColors[3] = mygrey1  # nose-left
        meshColors[4] = mygrey1  # fuselage1-top
        meshColors[5] = mygrey1  # fuselage1-top
        meshColors[6] = mygrey1  # fuselage1-right
        meshColors[7] = mygrey1  # fuselage1-right
        meshColors[8] = mygrey1  # fuselage1-left
        meshColors[9] = mygrey1  # fuselage1-left
        meshColors[10] = yellow  # fuselage1-bottom
        meshColors[11] = yellow  # fuselage1-bottom
        meshColors[12] = mygrey2  # fuselage2-top
        meshColors[13] = mygrey2  # fuselage2-top
        meshColors[14] = mygrey2  # fuselage2-right
        meshColors[15] = mygrey2  # fuselage2-right
        meshColors[16] = mygrey2  # fuselage2-left
        meshColors[17] = mygrey2  # fuselage2-left
        meshColors[18] = mygrey2  # fuselage2-bottom
        meshColors[19] = mygrey2  # fuselage2-bottom
        meshColors[20] = red  # flame-top
        meshColors[21] = red  # flame-right
        meshColors[22] = red  # flame-bottom
        meshColors[23] = red  # flame-left
        meshColors[24] = mygrey4  # top fin
        meshColors[25] = mygrey4  # top fin
        meshColors[26] = yellow  # bottom fin
        meshColors[27] = yellow  # bottom fin
        meshColors[28] = mygrey4  # right fin
        meshColors[29] = mygrey4  # right fin
        meshColors[30] = mygrey4  # left fin
        meshColors[31] = mygrey4  # left fin
        return points, meshColors

    def __points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([[points[0], points[1], points[2]],  # nose-top
                         [points[0], points[1], points[4]],  # nose-right
                         [points[0], points[3], points[4]],  # nose-bottom
                         [points[0], points[3], points[2]],  # nose-left
                         [points[1], points[2], points[5]],  # fuselage1-top
                         [points[2], points[5], points[6]],  # fuselage1-top
                         [points[1], points[4], points[5]],  # fuselage1-right
                         [points[8], points[4], points[5]],  # fuselage1-right
                         [points[2], points[3], points[6]],  # fuselage1-left
                         [points[3], points[6], points[7]],  # fuselage1-left     
                         [points[3], points[4], points[8]],  # fuselage1-bottom
                         [points[3], points[7], points[8]],  # fuselage1-bottom
                         [points[9], points[10], points[13]],  # fuselage2-top
                         [points[10], points[13], points[14]],  # fuselage2-top
                         [points[9], points[14], points[13]],  # fuselage2-right
                         [points[16], points[14], points[13]],  # fuselage2-right
                         [points[10], points[11], points[14]],  # fuselage2-left
                         [points[11], points[14], points[15]],  # fuselage2-left     
                         [points[11], points[12], points[16]],  # fuselage2-bottom
                         [points[11], points[15], points[16]],  # fuselage2-bottom
                         [points[21], points[17], points[18]],  # flame-top
                         [points[21], points[17], points[20]],  # flame-right
                         [points[21], points[19], points[20]],  # flame-bottom
                         [points[21], points[18], points[19]],  # flame-left
                         [points[22], points[23], points[24]],  # top fin
                         [points[22], points[24], points[25]],  # top fin
                         [points[26], points[27], points[28]],  # bottom fin
                         [points[26], points[28], points[29]],  # bottom fin
                         [points[30], points[31], points[32]],  # right fin
                         [points[30], points[32], points[33]],  # right fin
                         [points[34], points[35], points[36]],  # left fin
                         [points[34], points[36], points[37]],  # left fin
                         ])
        return mesh
