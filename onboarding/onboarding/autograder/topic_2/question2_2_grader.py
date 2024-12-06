from .. import utils
from ...questions.topic_2 import question2_2

def main(args=None):
    answers = question2_2.OnboardingTopic2_2()
    utils.verify_answer(3, answers.position_x_ned, 'Q2.2.a X coordinate NED')
    utils.verify_answer(2, answers.position_y_ned, 'Q2.2.b Y coordinate NED')
    utils.verify_answer(-5, answers.position_z_ned, 'Q2.2.c Z coordinate NED')


if __name__ == '__main__':
    main()
