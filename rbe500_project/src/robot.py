from sympy import Matrix as M
from sympy import symbols, sin, cos, pi, eye, zeros, sqrt, diff          # noqa
from sympy import init_printing, latex                                   # noqa
from sympy import Eq, solveset, simplify, collect, expand                # noqa
from sympy.matrices import rot_axis1, rot_axis2, rot_axis3               # noqa
from sympy.matrices.dense import randMatrix                              # noqa
from sympy.physics.mechanics import dynamicsymbols                       # noqa
from sympy.matrices.common import ShapeError                             # noqa
from sympy import pprint                                                 # noqa
from copy import deepcopy                                                # noqa


def simprint(matrix):
    pprint(simplify(matrix))


def headprint(heading):
    print(f'\n\n===== {heading} =====')


def skew(vector):
    # Skew-symmetric matrices allow us to use matrix multiplication to
    #     calculate cross products, among other things
    # Formally, it is a square matrix whose transpose equals its negative
    return M([
        [0,         -vector[2],  vector[1]],
        [vector[2],          0, -vector[0]],
        [-vector[1], vector[0],         0]])


def make_homogeneous(matrix):
    # Homogeneous matrices allow us to use matrix multiplication to apply
    #     both a rotation and a translation to a frame
    # Here, we'll take a rotation matrix or a translation matrix, and stamp
    #     it on top of a 4x4 identity matrix, thereby turning that rotation
    #     or translation into a homogeneous transformation matrix
    if matrix.shape == (3, 3):                 # 3x3: Rotation matrix
        no_translation = M([0, 0, 0])          # Add no translation
        top = M.hstack(matrix, no_translation)
    elif matrix.shape == (3, 1):               # 3x1: Translation matrix
        top = M.hstack(eye(3), matrix)         # Add no rotation
    else:
        # We did not get a rotation matrix nor a translation matrix!
        m = ('To make homogeneous, input matrix needs to be 3x3 '
             '(rotation) or 3x1 (translation). The given matrix is {}x{}.')
        raise TypeError(m.format(matrix.shape[0], matrix.shape[1]))

    # Add the bottom row of a homogeneous transformation matrix
    bottom = M([[0, 0, 0, 1]])
    return M.vstack(top, bottom)


def hinv(homogeneous_matrix):
    # Calculate the inverse of a homogeneous matrix
    rotation = homogeneous_matrix[:3, :3]
    translation = homogeneous_matrix[:3, 3]
    rotation_inverse = make_homogeneous(rotation.T)
    translation_inverse = make_homogeneous(-translation)
    return rotation_inverse @ translation_inverse


def chain_rule(expressions, q, qd, qdd):
    new_expressions = []
    for expression in expressions:  # Iterate through x/y/z dimensions
        partials = []
        for i in range(len(q)):     # Iterate through q1/q2/q3 joints
            partials.append(
                diff(expression, q[i]) * qd[i] +
                diff(expression, qd[i]) * qdd[i])
        new_expressions.append(partials)
    return simplify(M([sum(partial) for partial in new_expressions]))


def joint_transformation_dh(joint):
    # https://en.wikipedia.org/wiki/Denavit–Hartenberg_parameters
    a, θ, d, α = [joint[x] for x in ['a', 'θ', 'd', 'α']]
    return M([
        [cos(θ), -sin(θ)*cos(α),  sin(θ)*sin(α), a*cos(θ)],
        [sin(θ),  cos(θ)*cos(α), -cos(θ)*sin(α), a*sin(θ)],
        [0,              sin(α),         cos(α),        d],
        [0,                   0,              0,       1]])


def joint_transformation_poe(screw_axis, θ):
    # Assuming linear velocity on top
    id = eye(3)                     # Identity matrix
    v = M(screw_axis[:3])           # Linear translation of axis
    w = skew(screw_axis[3:])  # Angular rotation of axis (skewed)

    # Calculate pieces of the transformation matrix, courtesy of Rodrigues
    rotation = id + sin(θ) * w + (1 - cos(θ)) * w @ w
    translation = (id * θ + (1 - cos(θ)) * w + (θ - sin(θ)) * w @ w) @ v

    # Assemble the transformation matrix
    transformation = M.vstack(
        M.hstack(rotation, translation),  # Glue rotation and translation
        M([[0, 0, 0, 1]]))                # Add a homogeneous bottom
    return transformation


def check(robot):
    for frame in range(1, len(robot) + 1):
        print(f'\n---Frame {frame}---')
        simprint(robot.forward_kinematics_dh(frame))
        print('\n')
        simprint(robot.forward_kinematics_poe(frame))


class Robot:
    def __init__(self, dh_parameters=None,
                 screw_axes=None, joint_values=None, frames=None):
        # This class can take as input DH parameters and joint configuration,
        #     or parameters necessary for product of exponentials calculations
        if dh_parameters:  # We'll be using DH parameters
            self.dh = dh_parameters
        if screw_axes:     # We'll be using product of exponentials
            # We are always going to assume linear velocity (v) is on top,
            #     and angular velocity (ω) is on the bottom
            self.screw_axes = screw_axes
            self.frames = frames
            self.joint_values = joint_values

    def __len__(self):
        # Return the number of joints in the robot as its length
        try:
            return self.dh.shape[0]     # If DH parameters were given
        except AttributeError:
            return self.screw_axes.shape[1]  # If screw axes were given

    # ------ The below decorators are wrappers around internal variables ------
    @property
    def dh(self):
        return self._dh

    @dh.setter
    def dh(self, *args, **kwargs):
        # These are the symbols we're using and the order we're assuming
        dh_symbols = ['a', 'θ', 'd', 'α']

        # Populate the internal Denavit-Hartenberg parameters variable
        dh = []
        for joint in args[0]:
            if isinstance(joint, list) and len(joint) == 4:
                # If a list is given, assume values are in the order: a,θ,d,α
                dh.append(dict(zip(dh_symbols, joint)))
            elif all((isinstance(joint, dict), len(joint) == 4,
                     (x in joint for x in dh_symbols))):
                # If a dictionary of a,θ,d,α is given, just use that
                dh.append(joint)
        self._dh = dh

    @dh.deleter
    def dh(self):
        del self._dh

    @property
    def screw_axes(self):
        return self._screw_axes

    @screw_axes.setter
    def screw_axes(self, *args, **kwargs):
        self._screw_axes = M(args[0])

    @screw_axes.deleter
    def screw_axes(self):
        del self._screw_axes

    @property
    def frames(self):
        return self._frames

    @frames.setter
    def frames(self, *args, **kwargs):
        frames = [M(f) for f in args[0]]
        num_frames = len(frames)
        num_screw_axes = self.screw_axes.shape[1]
        if num_frames != num_screw_axes + 1:
            m = ('Given {} frames and {} joints, but there should be one more '
                 'frame than there are joints.')
            raise IndexError(m.format(num_frames, num_screw_axes))
        self._frames = frames

    @frames.deleter
    def frames(self):
        del self._frames

    @property
    def joint_values(self):
        return self._joint_values

    @joint_values.setter
    def joint_values(self, *args, **kwargs):
        given_joint_values = M([args[0]])
        num_joint_values = len(given_joint_values)
        num_screw_axes = self.screw_axes.shape[1]
        if num_joint_values != num_screw_axes:
            m = ('Given {} joint values (thetas) and {} screw axes, '
                 'but they need to match!')
            raise IndexError(m.format(num_joint_values, num_screw_axes))
        self._joint_values = given_joint_values

    @joint_values.deleter
    def joint_values(self):
        del self._joint_values
    # ------ The above decorators are wrappers around internal variables ------

    # Some default function definitions
    def forward_kinematics(self, *args, **kwargs):
        try:
            # Prioritize using Product of Exponentials approach
            return simplify(self.forward_kinematics_poe(*args, **kwargs))
        except AttributeError:
            # Try to use DH Parameters if screw axes were not given
            return simplify(self.forward_kinematics_dh(*args, **kwargs))

    def inverse_kinematics(self, *args, **kwargs):
        return self.numerical_inverse_position_kinematics(*args, **kwargs)

    def jacobian(self, *args, **kwargs):
        # https://en.wikipedia.org/wiki/Jacobian_matrix_and_determinant
        # Default to cross method for calculating Jacobian matrix
        return simplify(self.jacobian_cross(*args, **kwargs))

    def forward_kinematics_dh(self, current_frame=None, destination_frame=0):
        if current_frame is None:
            current_frame = len(self)
        # If the two frames are identical, return the identity matrix
        if current_frame == destination_frame:
            return eye(4)
        elif current_frame < destination_frame:
            # If going to a higher frame, do the opposite and take the inverse
            fk = self.forward_kinematics_dh(destination_frame, current_frame)
            return hinv(fk)

        # Get the transformation that corresponds to this frame
        current_transformation = joint_transformation_dh(
            self.dh[current_frame-1])

        # Return the product of the current transformation and the next
        next_transformation = self.forward_kinematics_dh(
            current_frame - 1, destination_frame)
        return next_transformation @ current_transformation

    def forward_kinematics_poe(self, current_frame=None, origin_frame=0):
        # This function will currently only calculate transformations with
        #     respect to the base frame

        # Use the end effector frame as the default, if not given
        if current_frame is None:
            current_frame = len(self)
        # If going to a higher frame, do the opposite and take the inverse)
        if current_frame < origin_frame:
            fk = self.forward_kinematics_poe(origin_frame, current_frame)
            return hinv(fk)

        # Initialize an identity matrix
        fk = eye(4)

        # Consecutively multiply exponentials of joints
        for joint_number in range(origin_frame, current_frame):
            screw_axis = self.screw_axes[:, joint_number]
            joint_value = self.joint_values[joint_number]
            fk = fk @ joint_transformation_poe(screw_axis, joint_value)

        if origin_frame == 0:
            # Multiply FK by the current frame's transformation matrix
            return fk @ self.frames[current_frame]
        else:
            # Calculate FK of the origin frame with respect to 0
            origin_fk = self.forward_kinematics_poe(origin_frame, 0)
            # Invert that
            origin_fk_inv = hinv(origin_fk)
            # Return the current frame's FK - the origin frame's FK
            return origin_fk_inv @ fk @ self.frames[current_frame]

    # Dubious
    def jacobian_cross(self, frame=None):
        # Make some dummy columns to stack onto
        jv, jw = zeros(3, 1), zeros(3, 1)

        # By default, we'll calculate the Jacobian for all joints
        if frame is None:
            frame = len(self)

        # First, we need the translation from the base to the end effector
        ee_translation = self.forward_kinematics(frame)[0:3, 3]
        # Then, iterate through all the joints, generating ω and v
        for joint_number in range(frame):
            transformation = self.forward_kinematics(joint_number)
            translation = ee_translation - transformation[0:3, 3]
            z_rotation = transformation[0:3, 2]

            jv = M.hstack(jv, z_rotation.cross(translation))  # Append v to Jv
            jw = M.hstack(jw, z_rotation)                     # Append ω to Jω

        return M.vstack(jv, jw)[:, 1:]  # Snip that dummy column

    # This doesn't presently work
    def jacobian_derivative(self, v_on_top=True):
        end_effector = self.forward_kinematics()[:3, 3]
        jv = end_effector.jacobian(self.joint_values)

        jw = zeros(3, 1)
        for joint_number in range(len(self)):
            transformation = self.forward_kinematics(joint_number)
            jw = M.hstack(jw, transformation[0:3, 2])
        jw = jw[:, 1:]

        return M.vstack(jv, jw) if v_on_top else M.vstack(jw, jv)

    # There is presently some small issue with this
    def numerical_inverse_position_kinematics(
            self, ee_target, acceptable_error=0.0001):
        # Make a clone of self, to mess around with joints carefree
        r = deepcopy(self)
        # Zero out the joint values, as an arbitrary starting point
        r.joint_values = zeros(1, len(r))
        # Grab the inverse of the velocity part of the Jacobian (for later)
        jv_inverse = r.jacobian()[:3, :].pinv()

        while True:  # Keep doing this until told otherwise
            # Get the end effector position, but evaluate it to a double/float,
            #     as the whole thing is an estimate anyway. This will speed it
            #     up. A lot.
            ee_position = r.forward_kinematics()[:3, 3].evalf()
            # Find the "error" distance between where the end effector is
            #     (with the current joint values), and where we want it to be
            error = abs((ee_target - ee_position).norm())

            # Print to console (for testing)
            print('End Effector Position:')
            pprint(ee_position)
            print(f'Error: {error}\n')

            if error <= acceptable_error:
                break  # We've reached a good enough value; let's quit

            # THIS IS WHERE THE MAGIC HAPPENS!
            # The Jacobian matrix lets us calculate end effector velocity from
            #     joint velocities, and its inverse lets us calculate joint
            #     velocities from end effector velocity
            # That inverse Jacobian can be thought of as a direction to move
            #     joints, given the desired direction of the end effector
            # The error (difference between the target and current position)
            #     gives us both a direction to move and a magnitude
            # Let's pop that into our Jacobian matrix, and we'll get a little
            #     closer every time
            joint_adjustment = jv_inverse @ (ee_target - ee_position)
            # Adjust the joints by the adjustment we just calculated
            r.joint_values += joint_adjustment.T  # Transposed to match shape

        return r.joint_values

    def singularities(self):
        jacobian = simplify(self.jacobian())
        jv = jacobian[:3, :]
        determinant = (jv @ jv.T).det()
        singularity_equation = Eq(determinant, 0)

        for joint_number, theta in enumerate(self.joint_values):
            print(f'Joint number: {joint_number}')
            singularities = solveset(singularity_equation, theta)
            yield singularities

    def analytical_jacobian(self):
        phi, the, psi = symbols('φ, θ, ψ')
        B_alpha = M([
            [0, -sin(phi), sin(the)*cos(phi)],
            [0,  cos(phi), sin(the)*sin(phi)],
            [1,         0,                 0]])
        T_alpha = M.vstack(
            M.hstack(eye(3), zeros(3)),
            M.hstack(zeros(3), B_alpha))

        return T_alpha.inv() @ self.jacobian()


def choose_robot(robot_id):
    if robot_id == 'rbe501_hw3':
        l1, l2, l3 = symbols('l1, l2, l3')
        q1, q2, q3 = dynamicsymbols('q1, q2, q3')
        return Robot(
            dh_parameters=[
                {'a':  0, 'θ': q1, 'd': l1, 'α': pi/2},
                {'a': l2, 'θ': q2, 'd':  0, 'α':    0},
                {'a': l3, 'θ': q3, 'd':  0, 'α':    0}],
            screw_axes=[
                [0,  l1,  l1],
                [0,   0,   0],
                [0,   0, -l2],
                [0,   0,   0],
                [0,  -1,  -1],
                [1,   0,   0]],
            frames=[
                [[1, 0,  0,     0],
                 [0, 1,  0,     0],
                 [0, 0,  1,     0],
                 [0, 0,  0,     1]],
                [[1, 0,  0,     0],
                 [0, 0, -1,     0],
                 [0, 1,  0,    l1],
                 [0, 0,  0,     1]],
                [[1, 0,  0,    l2],
                 [0, 0, -1,     0],
                 [0, 1,  0,    l1],
                 [0, 0,  0,     1]],
                [[1, 0,  0, l2+l3],
                 [0, 0, -1,     0],
                 [0, 1,  0,    l1],
                 [0, 0,  0,    1]]],
            joint_values=[q1, q2, q3])
    if robot_id == 'rbe501_hw5':
        l1, l2 = symbols('l1, l2')
        θ1, θ2 = dynamicsymbols('θ1, θ2')
        return Robot(
            screw_axes=[
                [0,   0],
                [0, -l1],
                [0,   0],
                [0,   0],
                [0,   0],
                [1,   1]],
            frames=[
                [[1, 0, 0,     0],
                 [0, 1, 0,     0],
                 [0, 0, 1,     0],
                 [0, 0, 0,     1]],
                [[1, 0, 0,    l1],
                 [0, 1, 0,     0],
                 [0, 0, 1,     0],
                 [0, 0, 0,     1]],
                [[1, 0, 0, l1+l2],
                 [0, 1, 0,     0],
                 [0, 0, 1,     0],
                 [0, 0, 0,     1]]],
            joint_values=[θ1, θ2])
    if robot_id == 'rbe501_final':
        a, b, c = symbols('a, b, c')
        q1, q2, q3 = dynamicsymbols('q1, q2, q3')
        return Robot(
            dh_parameters=[
                {'a': 0, 'θ': pi+q1, 'd':    a, 'α': pi/2},
                {'a': 0, 'θ':     0, 'd': b+q2, 'α': pi/2},
                {'a': 0, 'θ':  pi/2, 'd': c+q3, 'α':    0}],
            screw_axes=[
                [0,  0,  0],
                [0,  1,  0],
                [0,  0, -1],
                [0,  0,  0],
                [0,  0,  0],
                [1,  0,  0]],
            frames=[
                [[1,  0,  0,   0],
                 [0,  1,  0,   0],
                 [0,  0,  1,   0],
                 [0,  0,  0,   1]],
                [[-1, 0,  0,   0],
                 [0,  0,  1,   0],
                 [0,  1,  0,   a],
                 [0,  0,  0,   1]],
                [[-1, 0,  0,   0],
                 [0,  1,  0,   b],
                 [0,  0, -1,   a],
                 [0,  0,  0,   1]],
                [[0,  1,  0,   0],
                 [1,  0,  0,   b],
                 [0,  0, -1, a-c],
                 [0,  0,  0,   1]]],
            joint_values=[q1, q2, q3])
    if robot_id == 'rbe500_group_project':
        l1, l2, l3, l4 = symbols('l1, l2, l3, l4')
        θ1, θ2, d3 = dynamicsymbols('θ1, θ2, d3')
        return Robot(
            dh_parameters=[
                {'a': l2, 'θ': θ1, 'd':     l1, 'α':  0},
                {'a': l3, 'θ': θ2, 'd':      0, 'α':  0},
                {'a':  0, 'θ':  0, 'd': -d3-l4, 'α':  0}],
            screw_axes=[
                [0,   0,  0],
                [0, -l2,  0],
                [0,   0, -1],
                [0,   0,  0],
                [0,   0,  0],
                [1,   1,  0]],
            frames=[
                [[1, 0, 0,    0],
                 [0, 1, 0,    0],
                 [0, 0, 1,    0],
                 [0, 0, 0,    1]],
                [[1, 0, 0,    l2],
                 [0, 1, 0,     0],
                 [0, 0, 1,    l1],
                 [0, 0, 0,     1]],
                [[1, 0, 0, l2+l3],
                 [0, 1, 0,     0],
                 [0, 0, 1,    l1],
                 [0, 0, 0,     1]],
                [[1, 0, 0, l2+l3],
                 [0, 1, 0,     0],
                 [0, 0, 1, l1-l4],
                 [0, 0, 0,     1]]],
            joint_values=[θ1, θ2, d3])


def main():
    # Edit these formatting options to your liking
    init_printing(use_unicode=True, num_columns=300)

    # Set up the robot
    r = choose_robot('rbe500_group_project')  # noqa


if __name__ == "__main__":
    main()
