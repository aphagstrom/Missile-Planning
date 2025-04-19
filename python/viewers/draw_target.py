"""
draw target
    - Update history:
        7/12/23 - RWB
"""
import numpy as np
import pyqtgraph.opengl as gl


class DrawTarget:
    def __init__(self, state, window):
        # get points that define the non-rotated, non-translated mav and the mesh colors
        self.points, self.meshColors = self.__get_points()
        rotated_points = self.__rotate_points(self.points, np.identity(len(state.pos)))
        translated_points = self.__translate_points(rotated_points, state.pos.reshape((len(state.pos), 1)))
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.__points_to_mesh(translated_points)
        self.body = gl.GLMeshItem(
            vertexes=mesh,  # defines the triangular mesh (Nx3x3)
            vertexColors=self.meshColors,  # defines mesh colors (Nx1)
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
        window.addItem(self.body)  # add body to plot


    def update(self, state):
        # rotate and translate points defining mav
        rotated_points = self.__rotate_points(self.points, np.identity(len(state.pos)))
        translated_points = self.__translate_points(rotated_points, state.pos.reshape((len(state.pos), 1)))
        # convert North-East Down to East-North-Up for rendering
        R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
        translated_points = R @ translated_points
        # convert points to triangular mesh defined as array of three 3D points (Nx3x3)
        mesh = self.__points_to_mesh(translated_points)
        # draw MAV by resetting mesh using rotated and translated points
        self.body.setMeshData(vertexes=mesh, vertexColors=self.meshColors)

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
            Points that define the target, 
            and the colors of the triangular mesh
        """
        unit_length = 1000.0
        width = 1.0 * unit_length
        depth = 1.0 * unit_length
        height = 1.0 * unit_length
        
        # points are in NED coordinates
        points = np.array([ 
            [width/2., depth/2., height/2.],  # point 0
            [width/2., -depth/2., height/2.],  # point 1
            [-width/2., -depth/2., height/2.],  # point 2
            [-width/2., depth/2., height/2.],  # point 3
            [width/2., depth/2., -height/2.],  # point 4
            [width/2., -depth/2., -height/2.],  # point 5
            [-width/2., -depth/2., -height/2.],  # point 6
            [-width/2., depth/2., -height/2.],  # point 7
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

        meshColors = np.empty((12, 3, 4), dtype=np.float32)
        meshColors[0] = green  # bottom (+down)
        meshColors[1] = green  # bottom (+down)
        meshColors[2] = red  # top (-down)
        meshColors[3] = red  # top (-down)
        meshColors[4] = green  # right (+east)
        meshColors[5] = green  # right (+east)
        meshColors[6] = green  # left (-east)
        meshColors[7] = green  # left (-east)
        meshColors[8] = green  # front (+north)
        meshColors[9] = green  # front (+north)
        meshColors[10] = green  # back (-north)
        meshColors[11] = green  # back (-north)
        return points, meshColors

    def __points_to_mesh(self, points):
        """"
        Converts points to triangular mesh
        Each mesh face is defined by three 3D points
          (a rectangle requires two triangular mesh faces)
        """
        points = points.T
        mesh = np.array([
            [points[0], points[2], points[1]],  # bottom
            [points[0], points[2], points[3]],  # bottom
            [points[4], points[6], points[5]],  # top
            [points[4], points[6], points[7]],  # top
            [points[0], points[7], points[4]],  # right
            [points[0], points[7], points[3]],  # right
            [points[1], points[6], points[5]],  # left
            [points[1], points[6], points[2]],  # left
            [points[0], points[5], points[4]],  # front
            [points[0], points[5], points[1]],  # front
            [points[3], points[6], points[7]],  # back
            [points[3], points[6], points[2]],  # back
        ])
        return mesh
