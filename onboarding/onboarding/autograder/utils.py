GREEN = '\033[32m'
RED = '\033[31m'
RESET = '\033[0m'


def verify_answer(expected_condition, received_value, question_name):
    try:
        if callable(expected_condition):
            condition_met = expected_condition(received_value)
        else:
            condition_met = expected_condition == received_value

        if condition_met:
            print(f"{GREEN}PASSED{RESET} {question_name}")
        else:
            print(f"{RED}FAILED{RESET} {question_name}")
    except Exception as e:
        print(f"{RED}ERROR{RESET} {question_name}")
        print(e)
