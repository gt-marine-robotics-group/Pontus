from .. import utils
from ...questions.topic_2 import question2_1

def main(args=None):
    answers = question2_1.OnboardingTopic2_1()
    utils.verify_answer(2, answers.position_x_enu, 'Q2.1.a X coordinate ENU')
    utils.verify_answer(3, answers.position_y_enu, 'Q2.1.b Y coordinate ENU')
    utils.verify_answer(5, answers.position_z_enu, 'Q2.1.c Z coordinate ENU')


if __name__ == '__main__':
    main()
