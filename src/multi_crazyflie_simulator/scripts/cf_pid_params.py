class CF_pid_params():

    def __init__(self):

        # Recheck these values in the firmware
        ##################
        ## POSITION PID ##
        ##################

        # X POS
        self.KP_X = 2
        self.KI_X = 0
        self.KD_X = 0.0
        self.INT_MAX_X = 5000.0
        self.X_DT = 0.01

        # Y POS
        self.KP_Y = 2
        self.KI_Y = 0.0
        self.KD_Y = 0.0
        self.INT_MAX_Y = 5000.0
        self.Y_DT = 0.01

        # Z POS
        self.KP_Z = 2.0
        self.KI_Z = 0.5
        self.KD_Z = 0.0
        self.INT_MAX_Z = 5000.0
        self.Z_DT = 0.01


        ##################
        ## VELOCITY PID ##
        ##################

        # X VEL
        self.KP_VX = 25
        self.KI_VX = 1.0
        self.KD_VX = 0.0
        self.VX_DT = 0.01
        self.INT_MAX_VX = 5000.0
        self.MAX_ROLL = 20

        # Y VEL
        self.KP_VY = 25
        self.KI_VY = 1.0
        self.KD_VY = 0.0
        self.VY_DT = 0.01
        self.INT_MAX_VY = 5000.0
        self.MAX_PITCH = 20

        # Z VEL
        self.KP_VZ = 25.0
        self.KI_VZ = 15.0
        self.KD_VZ = 0.0
        self.VZ_DT = 0.01
        self.INT_MAX_VZ = 5000.0
        self.THRUST_SCALE = 1000

        #############
        ## ATT PID ##
        #############

        # PITCH
        self.KP_ROLL = 6.0
        self.KI_ROLL = 3.0
        self.KD_ROLL = 0.0
        self.INT_MAX_ROLL = 20
        self.ROLL_DT = 0.002

        # PITCH
        self.KP_PITCH = 6.0
        self.KI_PITCH = 3.0
        self.KD_PITCH = 0.0
        self.INT_MAX_PITCH = 20
        self.PITCH_DT = 0.002

        # YAw
        self.KP_YAW = 6.
        self.KI_YAW = 1.
        self.KD_YAW = 0.0
        self.INT_MAX_YAW = 360.0
        self.YAW_DT = 0.002

        self.MAX_ATT = 25.0

        #################
        ## Ang vel PID ##
        #################


        #####################
        ## NOISE gain = 1% ##
        #####################


        # WX VEL
        self.KP_WX = 250.0
        self.KI_WX = 500.0
        self.KD_WX = 0*2.5
        self.INT_MAX_WX = 33.3
        self.WX_DT = 0.002

        # WY VEL
        self.KP_WY = 250.0
        self.KI_WY = 500.0
        self.KD_WY = 0*2.5
        self.INT_MAX_WY = 33.3
        self.WY_DT = 0.002

        # WZ VEL
        self.KP_WZ = 120.
        self.KI_WZ = 16.7
        self.KD_WZ = 0.0
        self.INT_MAX_WZ = 166.7
        self.WZ_DT = 0.002
