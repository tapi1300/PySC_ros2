(define (domain house_domain)
(:requirements :strips :typing :adl :fluents :durative-actions)

;; Types ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:types
robot
zone room corridor - place
object
);; end Types ;;;;;;;;;;;;;;;;;;;;;;;;;

;; Predicates ;;;;;;;;;;;;;;;;;;;;;;;;;
(:predicates

(robot_at ?r - robot ?p - place)
(object_at ?o - object ?p - place)
(robot_available ?r - robot)
(not_robot_at_zone ?r - robot)
(zone_at ?z - zone ?p - place)
(robot_at_zone ?r - robot ?z - zone)
(free ?r - robot)
(object_at_robot ?r - robot ?o - object)
(connected ?p1 - place ?p2 - place)
);; end Predicates ;;;;;;;;;;;;;;;;;;;;
;; Functions ;;;;;;;;;;;;;;;;;;;;;;;;;
(:functions

);; end Functions ;;;;;;;;;;;;;;;;;;;;
;; Actions ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(:durative-action move
    :parameters (?r - robot ?from ?to - place)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?from))
        (at start(robot_available ?r))
        (over all (not_robot_at_zone ?r))
        (over all (connected ?from ?to))
        )
    :effect (and
        (at start (not(robot_at ?r ?from)))
        (at end (robot_at ?r ?to))
        (at start (not(robot_available ?r)))
        (at end (robot_available ?r))
    )
)

(:durative-action move_to_zone
    :parameters (?r - robot ?p - place ?z - zone)
    :duration ( = ?duration 2)
    :condition (and
        (at start(robot_at ?r ?p))
        (at start(robot_available ?r))
        (at start (not_robot_at_zone ?r))
        (over all (zone_at ?z ?p))
        )
    :effect (and
        (at start(not(robot_available ?r)))
        (at end (robot_available ?r))
        (at end (not(not_robot_at_zone ?r)))
        (at end(robot_at ?r ?z))
    )
)

(:durative-action move_out_zone
    :parameters (?r - robot ?p - place ?z - zone)
    :duration ( = ?duration 2)
    :condition (and
        (at start(robot_at ?r ?p))
        (at start(robot_at ?r ?z))
        (at start(robot_available ?r))
        (over all (zone_at ?z ?p))
        )
    :effect (and
        (at start(not(robot_available ?r)))
        (at end (robot_available ?r))
        (at end (not_robot_at_zone ?r))
        (at end (not(robot_at ?r ?z)))
        (at end (robot_at ?r ?p))
    )
)

(:durative-action pick
    :parameters (?r - robot ?p - place ?o - object)
    :duration ( = ?duration 10)
    :condition (and
        (at start (robot_at ?r ?p))
        (at start (object_at ?o ?p))
        (at start (robot_available ?r))
        (at start (free ?r))
        )
    :effect (and
        (at start(not(robot_available ?r)))
        (at end (robot_available ?r))
        (at end (not(object_at ?o ?p)))
        (at end (not(free ?r)))
        (at end (object_at_robot ?r ?o))
    )
)

(:durative-action drop
    :parameters (?r - robot ?p - place ?o - object)
    :duration ( = ?duration 5)
    :condition (and
        (at start(robot_at ?r ?p))
        (at start(robot_available ?r))
        (at start (object_at_robot ?r ?o))
        )
    :effect (and
        (at start(not(robot_available ?r)))
        (at end (robot_available ?r))
        (at end(not(object_at_robot ?r ?o)))
        (at end (free ?r ))
        (at end (object_at ?o ?p))
    )
)

)