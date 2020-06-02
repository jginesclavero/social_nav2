(define (domain escort)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
waypoint
agent_id
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?wp - waypoint)
(connected ?wp1 ?wp2 - waypoint)
;;(patrolled ?wp - waypoint)
(accompanied ?id - agent_id)

);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(:durative-action escort
    :parameters (?r - robot ?id - agent_id)
    :duration ( = ?duration 5)
    :condition (and
       )
    :effect (and
        (at end(accompanied ?id))
    )
)

(:durative-action move
    :parameters (?r - robot ?wp1 ?wp2 - waypoint)
    :duration ( = ?duration 5)
    :condition (and
        (at start(connected ?wp1 ?wp2))
        (at start(robot_at ?r ?wp1))
        )
    :effect (and
        (at start(not(robot_at ?r ?wp1)))
        (at end(robot_at ?r ?wp2))
    )
)

);; end Domain ;;;;;;;;;;;;;;;;;;;;;;;;
