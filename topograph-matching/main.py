import x_view as xv
import os


def main():
    dim = 3
    G1 = xv.get_topograph1(dim=dim)
    G2 = xv.get_topograph2(dim=dim)

    f_inner = xv.force.Hooke(k=100)
    f_outer = xv.force.Hooke(k=20)
    i = xv.integrator.EE(dt=0.02, mu=0.001)

    scene = xv.MatchingScene([G1, G2], integrator=i, inner_force=f_inner, outer_force=f_outer, dim=dim)

    output_base_name = "fig"

    draw_iter = 0
    draw_every = 1
    max_it = 1000
    for step in range(max_it):

        if draw_every > 0 and step % draw_every == 0:
            drawer = xv.TopoGraphPlotter(scene.topograph_list)
            drawer.font_size = 14
            drawer.label_color = "#FF8400"
            drawer.outer_connections_visible = True
            drawer.edge_color_type = xv.EdgeColor.LENGTH_DEVIATION

            filename = os.path.join(output_base_name, str(draw_iter).zfill(4) + ".png")
            drawer.save(filename)
            draw_iter += 1

        scene.step()
        xv.printProgressBar(iteration=step, total=max_it)

if __name__ == '__main__':
    main()
