import numpy as np

from pontus_autonomy.tasks.base_task import BaseTask
from pontus_autonomy.helpers.search_helper import SearchHelper, SearchConditions

class SearchWrapper(BaseTask):

    def __init__(
        self,
        search_angle_min = -np.pi/8,
        search_angle_max = np.pi/8,
        terminating_condition = SearchConditions.GATE,
        num_slalom_rows_required = 1):

        super().__init__("search_wrapper_task")

        self.search_helper = SearchHelper(self,
                                          search_angle_min,
                                          search_angle_max,
                                          terminating_condition,
                                          num_slalom_rows_required)

        self.search_helper.start_searching()

        self.timer = self.create_timer(0.1, self.check_search)

    def check_search(self):
        if self.search_helper.complete:
            self.complete(self.search_helper.success)