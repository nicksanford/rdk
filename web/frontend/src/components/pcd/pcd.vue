<script setup lang="ts">
import {
  CameraClient,
  Client,
  commonApi,
} from '@viamrobotics/sdk';
import { $ref } from 'vue/macros';
import { toast } from '../../lib/toast';
// import PCD from './pcd-view.vue';
import Slam2dRender from '../slam-2d-render.vue';

const props = defineProps<{
  cameraName: string;
  resources: commonApi.ResourceName.AsObject[];
  client: Client;
}>();

let pcdExpanded = $ref(false);
let pointcloud = $ref<Uint8Array | undefined>();
let pointCloudUpdateCount = $ref(0);

const renderPCD = async () => {
  try {
    pointcloud = await new CameraClient(props.client, props.cameraName).getPointCloud();
    pointCloudUpdateCount += 1;
  } catch (error) {
    toast.error(`Error getting point cloud: ${error}`);
  }
};

const togglePCDExpand = () => {
  pcdExpanded = !pcdExpanded;
  if (pcdExpanded) {
    renderPCD();
  }
};
</script>

<template>
  <div class="pt-4">
    <div class="flex gap-2 align-top">
      <v-switch
        :label="pcdExpanded ? 'Hide Point Cloud Data' : 'View Point Cloud Data'"
        :value="pcdExpanded ? 'on' : 'off'"
        @input="togglePCDExpand"
      />

      <v-tooltip
        text="When turned on, point cloud will be recalculated."
        location="top"
      >
        <v-icon
          name="info-outline"
        />
      </v-tooltip>
    </div>

    <!-- <PCD
      v-if="pcdExpanded"
      :resources="resources"
      :pointcloud="pointcloud"
      :camera-name="cameraName"
      :client="client"
    /> -->
  </div>
  <Slam2dRender
    :point-cloud-update-count="pointCloudUpdateCount"
    :pointcloud="pointcloud"
    :name="cameraName"
    :resources="resources"
    :dest-exists="false"
    :axes-visible="true"
    @click="(_) => {}"
  />
</template>
