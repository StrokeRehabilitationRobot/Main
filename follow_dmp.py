



def way_points(path):
    movement = []
    count = 0
    while count < len(path)-2:

        p0 = path[count]
        p1 = path[count +1]
        p2 = path[count +2]

        m1 = None
        m2 = None
        dir = None
        # Get the two slopes
        if p1[0] == p0[0]:
            m1 = (p1[1]-p0[1])*float("inf")
        else:
            m1 = (p1[1] - p0[1])/(p1[0] - p0[0])

        if p2[0] == p1[0]:
            m2 = (p2[1] - p1[1]) * float("inf")
        else:
            m2 = (p2[1] - p1[1]) / (p2[0] - p1[0])


        # compare the slopes
        if m1 == float("inf") and m2 == float("inf"):
            dir = "N"
        elif m1 == -float("inf") and m2 == -float("inf"):
            dir = "S"
        elif m1 == 0 and m2 == 0:
            if p2[0] > p1[0]:
                dir = "W"
            else:
                dir = "E"
        elif m1 == 0 and m2 == float("inf"):
            if p2[0] > p0[0]:
                dir = "WN"
            else:
                dir = "EN"
        elif m1 == 0 and m2 == -float("inf"):
            if p2[0] > p0[0]:
                dir = "WS"
            else:
                dir = "ES"
        elif m2 == 0 and m1 == float("inf"):
            if p2[0] > p0[0]:
                dir = "NW"
            else:
                dir = "NE"
        elif m2 == 0 and m1 == -float("inf"):
            if p2[0] > p0[0]:
                dir = "SW"
            else:
                dir = "SE"


        movement.append(( dir, p0, p2 ))
        count += 2


    return movement



