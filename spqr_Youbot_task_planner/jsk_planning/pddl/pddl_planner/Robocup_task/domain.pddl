(define (domain spqr-task-planning)
  (:requirements :typing :action-costs)	
	(:predicates 	
		
		(location ?loc)				
	
		(OBJ ?o)
		
		(ROBOT ?r)		
		(SLOT ?sl)
		(emptySLOT ?sl)
		
		(obj-at-location ?o ?loc)
		(onSLOT ?o ?sl)
		
		(robot-at-location ?r ?loc)		
		(edge ?loc1 ?loc2)
	
)

(:action TAKE-OBJ-FROM-LOCATION
  :parameters
   (?r
    ?o
    ?loc
    ?sl
  )
  :precondition
   (and (ROBOT ?r)(location ?loc)(OBJ ?o)(SLOT ?sl)(obj-at-location ?o ?loc)(robot-at-location ?r ?loc)(emptySLOT ?sl))
  :effect
   (and (not(obj-at-location ?o ?loc))(not(emptySLOT ?sl))(onSLOT ?o ?sl))
) 
	
 (:action DROP-OBJECT
  :parameters
   (?r
    ?o
    ?loc
    ?sl
  )
  :precondition
   (and (ROBOT ?r)(location ?loc)(OBJ ?o)(SLOT ?sl)(onSLOT ?o ?sl)(robot-at-location ?r ?loc)(not(emptySLOT ?sl)))
  :effect
   (and (obj-at-location ?o ?loc)(emptySLOT ?sl)(not(onSLOT ?o ?sl)))
) 

(:action MOVE
  :parameters
   (?r
    ?pFrom
    ?pTo
  )
  :precondition
   (and (ROBOT ?r)(location ?pFrom)(location ?pTo)(edge ?pFrom ?pTo)(robot-at-location ?r ?pFrom)(not(robot-at-location ?r ?pTo)))
  :effect
   (and (not(robot-at-location ?r ?pFrom))(robot-at-location ?r ?pTo))
 )
)
