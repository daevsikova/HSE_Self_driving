from gym_duckietown.tasks.task_solution import TaskSolution
import numpy as np
import cv2

class DontCrushDuckieTaskSolution(TaskSolution):
    def __init__(self, generated_task):
        super().__init__(generated_task)

    def check_yellow(self, img):
        mask = ((img[..., 0] >= 230) & (img[..., 1] >= 230)).astype(int)
        return mask.mean()

    def step(self, env, vel, angle):
        img, reward, done, info = env.step([vel, angle])
        env.render()
        return img, reward, done, info

    def turn(self, env, n=7, angle=40):
        self.step(env, 1, angle)
        for _ in range(n):
            self.step(env, 1, 0)
        self.step(env, 1, -angle)

    def solve(self):
        env = self.generated_task['env']
        # getting the initial picture
        img, _, _, _ = env.step([0,0])
        eps = 1e-4
        turned = False
        max_straight_steps = 100

        for _ in range(max_straight_steps):
            img, _, _, _ = self.step(env, 1, 0)

            if self.check_yellow(img) > eps:
                if not turned:
                    # need to turn left
                    self.turn(env, n=10, angle=40)
                    turned = True
                # else will go straight
            
            else:
                if turned:
                    # need to turn right
                    self.turn(env, n=10, angle=-40)
                    turned = False
                # else will go straight
