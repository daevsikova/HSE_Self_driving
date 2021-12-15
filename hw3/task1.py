from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def check_yellow(self, img):
        mask = ((img[..., 0] >= 230) & (img[..., 1] >= 230)).astype(int)
        return mask.mean()

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0,0])
        eps = 1e-4
        
        condition = True
        while condition:
            img, reward, done, info = env.step([1, 0])
            # img in RGB
            # add here some image processing
            if self.check_yellow(img) < eps:
                condition = True
            else:
                condition = False
            env.render()
