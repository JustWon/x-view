import x_view as xv
import os
import random
import numpy as np


def main():

    map = xv.generate_map(10, 5, 3)
    return

    dim = 2
    G1 = xv.get_restricted_topograph1(edge_color='#2B3EAB', node_color='#455BD9')
    G2 = xv.get_restricted_topograph2(edge_color='#69D14D', node_color='#33A614')

    f_inner = xv.force.Hooke(k=1000)
    f_outer = xv.force.Hooke(k=40)
    i = xv.integrator.EE(dt=0.01, mu=0.1)

    scene = xv.MatchingScene([G1, G2], integrator=i, inner_force=f_inner, outer_force=f_outer, dim=dim)

    output_base_name = "fig"

    draw_iter = 0
    draw_every = 2
    max_it = 2000
    for step in range(max_it):

        if step == 0:
            for a in range(20):
                drawer = xv.TopoGraphDrawer(scene.topograph_list)
                drawer.font_size = 14
                drawer.outer_connections_visible = True
                drawer.edge_color_type = xv.EdgeColor.EDGE_COLOR

                filename = os.path.join(output_base_name, str(draw_iter).zfill(4) + ".png")
                drawer.save(filename)
                draw_iter += 1

        if draw_every > 0 and step % draw_every == 0:
            drawer = xv.TopoGraphDrawer(scene.topograph_list)
            drawer.font_size = 14
            drawer.outer_connections_visible = True
            drawer.edge_color_type = xv.EdgeColor.EDGE_COLOR

            filename = os.path.join(output_base_name, str(draw_iter).zfill(4) + ".png")
            drawer.save(filename)
            draw_iter += 1

        scene.step()

        xv.printProgressBar(iteration=step, total=max_it)


if __name__ == '__main__':
    np.random.seed(20)
    random.seed(20)
    main()
