from ...autograder import utils
from ...questions.topic_1 import question1_1

def main(args=None):
    answers = question1_1.Question_1_1()
    utils.verify_answer(1, answers.num_nodes, 'Q1.1.a Nodes')
    utils.verify_answer("/onboarding_node_q1_1", answers.first_node_name, 'Q1.1.b Nodes Names')
    utils.verify_answer(5, answers.num_topics, 'Q1.1.c Topics')
    utils.verify_answer("geometry_msgs/msg/Twist", answers.topic_message_type, 'Q1.1.d Topic Info')
    utils.verify_answer("mrg", answers.string_message, 'Q1.1.e Topic Echo')


if __name__ == '__main__':
    main()
