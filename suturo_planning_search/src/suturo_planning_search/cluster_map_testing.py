from suturo_planning_search.cluster_map import ClusterRegions
from suturo_planning_search.cell import Cell

__author__ = 'pmania'

def test_field1():
    c = Cell()
    c.set_free()
    field = [[c for x in xrange(15)] for x in xrange(15)]

    field[0][6] = Cell()
    field[0][6].set_unknown()
    field[0][7] = Cell()
    field[0][7].set_unknown()
    field[0][8] = Cell()
    field[0][8].set_unknown()
    field[2][6] = Cell()
    field[2][6].set_unknown()
    field[2][7] = Cell()
    field[2][7].set_unknown()
    field[2][8] = Cell()
    field[2][8].set_unknown()
    return field


def test_field2():
    size = 50
    field = [[Cell() for x in xrange(size)] for x in xrange(size)]

    for row in field:
        for c in row:
            c.set_unknown()
    return field


def test_field3():
    c = Cell()
    c.set_free()
    field = [[c for x in xrange(15)] for x in xrange(15)]

    field[0][6] = Cell()
    field[0][6].set_unknown()
    field[0][7] = Cell()
    field[0][7].set_unknown()
    field[0][8] = Cell()
    field[0][8].set_unknown()

    field[1][6] = Cell()
    field[1][6].set_unknown()
    field[2][6] = Cell()
    field[2][6].set_unknown()

    field[1][8] = Cell()
    field[1][8].set_unknown()
    field[2][8] = Cell()
    field[2][8].set_unknown()
    return field


def test_field4():
    c = Cell()
    c.set_free()
    field = [[c for x in xrange(15)] for x in xrange(15)]

    # field[0][6] = Cell()
    # field[0][6].set_unknown()
    # field[0][7] = Cell()
    # field[0][7].set_unknown()
    field[0][8] = Cell()
    field[0][8].set_unknown()

    field[1][7] = Cell()
    field[1][7].set_unknown()
    field[2][6] = Cell()
    field[2][6].set_unknown()

    # field[1][8] = Cell()
    # field[1][8].set_unknown()
    # field[2][8] = Cell()
    # field[2][8].set_unknown()
    return field

cm = ClusterRegions()
cm.set_field(test_field4())
cm.print_field()

print "Returned regions: " + str(cm.group_regions())
print "Grouping done"
cm.print_segmented_field()
c = cm.get_result_map()

regions = cm.get_result_regions()
print "Region count: " + str(len(regions))
for r in regions:
    print r
    print "Distance of region to (0,0): " + str(r.euclidean_distance_to_avg(0,0))
cubified_field = cm.cubify_regions()
cm.print_2d_field(cubified_field)
cm.split_and_cubify_regions()