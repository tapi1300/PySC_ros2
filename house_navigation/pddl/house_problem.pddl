(define (problem house_problem)
(:domain house_domain)
(:objects 
    r1 r2 r3 - room
    c1 c2 c3 - corridor
    o1 o2 - object
    r - robot
    z1 - zone
)
(:init 

    (robot_at r r1)
    (robot_available r)
    (not_robot_at_zone r)
    (free r)
    (object_at o1 r3)
    (connected r2 r3)
    (connected r3 r2)
    (connected r1 c1)
    (connected c1 r1)
    (connected r2 c1)
    (connected c1 r2)
    (zone_at z1 r2)
)

(:goal 
    (and
        (object_at o1 z1)
        (robot_at r r1)
    )
)
)