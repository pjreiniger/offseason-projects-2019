package frc.robot.subsystems;

import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;
import frc.robot.subsystems.requests.RequestList;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.List;

/**
 * A class to handle subsystem requests and the robot as a whole.
 * Should be in robot.java, but because its such a core class,
 * this will mitigate the damage from it.
 */
public class Superstructure extends Subsystem {
  public Lift lift;

  private Superstructure() {
    lift = Lift.getInstance();

    queuedRequests = new ArrayList<>(0);
  }

  private static Superstructure instance = null;

  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }
    return instance;
  }

  private RequestList activeRequests;
  private ArrayList<RequestList> queuedRequests;
  private Request currentRequest;

  private boolean newRequests = false;
  private boolean activeRequestsCompleted = false;
  private boolean allRequestsCompleted = false;

  public boolean requestsCompleted() {
    return allRequestsCompleted;
  }

  private void setActiveRequests(RequestList requests) {
    activeRequests = requests;
    newRequests = true;
    activeRequestsCompleted = false;
    allRequestsCompleted = false;
  }

  private void setQueuedRequests(List<RequestList> requests) {
    queuedRequests.clear();
    queuedRequests = new ArrayList<>(requests.size());
    for (RequestList list : requests) {
      queuedRequests.add(list);
    }
  }

  private void setQueuedRequests(RequestList requests) {
    queuedRequests.clear();
    queuedRequests.add(requests);
  }

  public void request(Request r) {
    setActiveRequests(new RequestList(Arrays.asList(r), false));
    setQueuedRequests(new RequestList());
  }

  public void request(Request active, Request queue) {
    setActiveRequests(new RequestList(Arrays.asList(active), false));
    setQueuedRequests(new RequestList(Arrays.asList(queue), false));
  }

  public void request(RequestList activeList) {
    setActiveRequests(activeList);
    setQueuedRequests(new RequestList());
  }

  public void request(RequestList activeList, RequestList queuedList) {
    setActiveRequests(activeList);
    setQueuedRequests(queuedList);
  }

  public void addActiveRequest(Request r) {
    activeRequests.add(r);
    newRequests = true;
    activeRequestsCompleted = false;
    allRequestsCompleted = false;
  }

  public void queue(Request r) {
    queuedRequests.add(new RequestList(Arrays.asList(r), false));
  }

  public void queue(RequestList list) {
    queuedRequests.add(list);
  }

  public void replaceQueue(Request r) {
    setQueuedRequests(new RequestList(Arrays.asList(r), false));
  }

  public void replaceQueue(RequestList list) {
    setQueuedRequests(list);
  }

  public void replaceQueue(List<RequestList> lists) {
    setQueuedRequests(lists);
  }

  private final Loop loop = new Loop() {

    @Override
    public void onStart(double timestamp) {
      stop();
    }

    @Override
    public void onLoop(double timestamp) {
      if (newRequests) {
        if (activeRequests.isParallel()) {
          boolean allActivated = true;
          for (Iterator<Request> iterator = activeRequests.getRequests().iterator(); 
              iterator.hasNext();) {
                Request request = iterator.next();
                boolean allowed = request.allowed();
                allActivated &= allowed;
                if (allowed) {
                  request.act();
                }
          }
          newRequests = !allActivated;
        } else {
          if (activeRequests.isEmpty()) {
            activeRequestsCompleted = true;
            return;
          }
          currentRequest = activeRequests.remove();
          currentRequest.act();
          newRequests = false;
        }
      }
      if (activeRequests.isParallel()) {
        boolean done = true;
        for (Request r : activeRequests.getRequests()) {
          done &= r.isFinished();
        }
        activeRequestsCompleted = done;
      } else if (currentRequest.isFinished()) {
        if (activeRequests.isEmpty()) {
          activeRequestsCompleted = true;
        } else if (activeRequests.getRequests().get(0).allowed()) {
          newRequests = true;
          activeRequestsCompleted = false;
        }
      }
    }

    @Override
    public void onStop(double timestamp) {

    }

  };

  public synchronized void sendManualRequest(double liftOutput) {
    RequestList list = RequestList.emptyList();
    if (liftOutput != 0) {
      list.add(lift.openLoopRequest(liftOutput));
    }
    if (!list.isEmpty()) {
      request(list);
    }
  }

  public RequestList idleRequest() {
    return new RequestList(Arrays.asList(lift.openLoopRequest(0.0)), true);
  }

  public void disabledState() {

  }


  @Override
  public void stop() {
    setActiveRequests(idleRequest());
  }

  @Override
  public void outputTelemetery() {

  }

  @Override
  public void registerEnabledLooper(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

}