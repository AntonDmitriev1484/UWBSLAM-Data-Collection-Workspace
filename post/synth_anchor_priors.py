
# Slightly errored pose of each anchor in the slam frame, at varied timestamps, to be used as a prior for graph optimization

# Anchor positions in SLAM frame
# Anchor 2
#  GT  [-0.52073365 -0.78536964 -0.23552549]
# Anchor 3
#  GT  [ 2.60637062  2.67963209 -0.45687288]
# Anchor 4
#  GT  [ 3.30309063  0.11910768 -0.36471837]

import numpy as np

# Just 6 random timestmaps I drew from the data

# def compute_synth_visual_anchor_priors(START, END):

#     # All coordinates in SLAM frame

#     id_to_gt = {
#         2: [-0.52073365, -0.78536964, -0.23552549],
#         3: [ 2.60637062,  2.67963209, -0.45687288],
#         4: [ 3.30309063,  0.11910768, -0.36471837]
#     }

#     id_to_data = {
#         2: [
#             (1757624988.5207834, [-1, -0.78, -0.23]),
#             (1757624906.3436587, [-0.7, -0.5, -0.5]),
#         ],
#         3: [
#             (1757624964.2945166, [2.6, 2.6, -1]),
#             (1757624922.035635,  [2.3, 2.3, 0]),
#         ],
#         4: [
#             (1757624986.5869217, [2.7, 0.5, -0.36471837]),
#             (1757624937.0911634, [3.1, 0.4, -0.7]),
#         ],
#     }


#     synth_visual_anchor_priors = []
#     for id, arr in id_to_data.items():
#         for t, position in arr:

#             position = np.array(position)
#             gt_position = np.array(id_to_gt[id])
#             T_body_to_slamworld = np.eye(4)
#             T_body_to_slamworld[:3,3] = np.array(position)

#             print(f"Anchor {id} synth visual prior error: {np.linalg.norm(position - gt_position)}")
#             synth_visual_anchor_priors.append(
#                         {               
#                             "t": t,
#                             "type": "synth_visual_anchor_prior",
#                             "src": id,
#                             "T_body_world": T_body_to_slamworld
#                             }
#             )


#     return synth_visual_anchor_priors


def compute_synth_visual_anchor_priors(START, END, stdev, N):
    rng = np.random.default_rng()

    id_to_data = {}

    id_to_gt = { # Took these values from plot_single.py in GTSAM
        2: [-0.52073365, -0.78536964, -0.23552549],
        3: [ 2.60637062,  2.67963209, -0.45687288],
        4: [ 3.30309063,  0.11910768, -0.36471837]
    }

    for anchor_id, gt_pos in id_to_gt.items():
        gt_pos = np.asarray(gt_pos)

        # Uniform timestamps
        timestamps = rng.uniform(START, END, size=N)

        # 3D Gaussian samples
        positions = rng.normal(
            loc=gt_pos,
            scale=stdev,
            size=(N, 3),
        )

        id_to_data[anchor_id] = list(zip(timestamps, positions.tolist()))

    synth_visual_anchor_priors = []
    for id, arr in id_to_data.items():
        for t, position in arr:

            position = np.array(position)
            gt_position = np.array(id_to_gt[id])
            T_body_to_slamworld = np.eye(4)
            T_body_to_slamworld[:3,3] = np.array(position)

            print(f"Anchor {id} synth visual prior error: {np.linalg.norm(position - gt_position)}")
            synth_visual_anchor_priors.append(
                        {               
                            "t": t,
                            "type": "synth_visual_anchor_prior",
                            "src": id,
                            "T_body_world": T_body_to_slamworld
                            }
            )


    return synth_visual_anchor_priors
