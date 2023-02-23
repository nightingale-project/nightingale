from nightingale_dispatcher.navigate_task import NavigateTask
import time
from nightingale_dispatcher.task import Task, TaskCodes
import rospy



if __name__ == "__main__":
    rospy.init_node("room_runner_tester")
    rr = NavigateTask()
    #assert rr.execute("room0", "bedside") == TaskCodes.SUCCESS
    time.sleep(3)
    assert rr.execute("stock", "default") == TaskCodes.SUCCESS
    time.sleep(3)
    assert rr.execute("room0", "bedside") == TaskCodes.SUCCESS
    rospy.loginfo("done")

