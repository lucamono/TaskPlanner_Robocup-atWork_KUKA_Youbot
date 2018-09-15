(define (problem spqr-planning)
   (:domain spqr-task-planning)
	(:objects
	    youbot
	    slot1
	    slot2
	    slot3

	    WS05
	    WS06
	    WPS
	    WPS_1
	    WPS_2
	    WPS_3
	    WPS_4
	    WPS_5

	    AXIS
	    BEARING
	    S40_40_B1
	    M20
	    S40_40_B

	)
	(:init
	    (ROBOT youbot)
	    (SLOT slot1)
	    (SLOT slot2)
	    (SLOT slot3)

	    (OBJ AXIS)
	    (OBJ BEARING)
	    (OBJ S40_40_B1)
	    (OBJ M20)
	    (OBJ S40_40_B)

	    (location WS05)
	    (location WS06)
	    (location WPS)
	    (location WPS_1)
	    (location WPS_2)
	    (location WPS_3)
	    (location WPS_4)
	    (location WPS_5)

	    (edge WPS WPS_1)
	    (edge WPS_1 WPS_2)
	    (edge WPS_2 WPS_3)
	    (edge WPS_3 WPS_4)
	    (edge WPS_4 WPS_5)
	    (edge WPS_5 WS05)
	    (edge WS05 WS06)
	    (edge WS06 WS05)

	    (emptySLOT slot1)
	    (emptySLOT slot2)
	    (emptySLOT slot3)

	    (robot-at-location youbot WPS)

	    (obj-at-location AXIS WS05)
	    (obj-at-location BEARING WS05)
	    (obj-at-location S40_40_B1 WS05)
	    (obj-at-location M20 WS05)
	    (obj-at-location S40_40_B WS05)

	)
	(:goal
               (and 
          	    (obj-at-location AXIS WS06)
          	    (obj-at-location BEARING WS06)
          	    (obj-at-location S40_40_B1 WS06)
          	    (obj-at-location M20 WS06)
          	    (obj-at-location S40_40_B WS06)
               )
	)
)